/* =============================================================================
 *  audio_capture.c — 8kHz 音频采集实现
 * =============================================================================
 *  见 audio_capture.h 顶部的设计说明（为什么必须改采样率）。
 *
 *  实现要点（三个"必须"，漏一个就采不到正确的波形）
 *  ---------------------------------------------------------------------------
 *  ① **TIM2 的 TRGO 必须选 Update 事件**
 *       TIM2 配成 8kHz 的 PWM 后，每个周期结束时产生 Update 事件；
 *       把这个事件选为 TRGO（Trigger Output），ADC 才有"外部触发源"可用。
 *       如果 TRGO 默认是 Reset/Enable，ADC 收不到触发 → 一个样本都不采。
 *
 *  ② **ADC 的外部触发必须选 TIM2_TRGO、边沿选上升沿**
 *       F1 的 ADC 支持多种外部触发，我们要的是 `ADC_EXTERNALTRIGCONV_T2_TRGO`。
 *       同时 `ExternalTrigConvEdge` 必须是**上升沿**（不是 NONE），
 *       否则触发被禁用 → 同样一个样本都不采。
 *
 *  ③ **DMA 必须是 Circular（循环）+ 外设到内存**
 *       ADC 每个样本产生一次 DMA 请求，循环模式下 DMA 搬满一圈后自动回到开头，
 *       所以缓冲区永远是"最近 AUDIO_DMA_BUF_LEN 个样本"的滚动窗口，
 *       CPU 不需要重新启动传输。
 *
 *  ⚠ 这个文件**绕过 CubeMX** 直接配置寄存器级参数。
 *    好处：不用在 CubeMX 里点 4 个页面，且参数一目了然。
 *    代价：如果以后在 CubeMX 里重新 GENERATE CODE，这里的配置不会被恢复 ——
 *         重新生成后只需再次调用 audio_init() 即可（配置在运行时做，不依赖 CubeMX）。
 * ========================================================================== */

#include "audio_capture.h"
#include "main.h"

/* ---- 内部状态 ---------------------------------------------------------- */
static DMA_HandleTypeDef  s_hdma_adc;          /* ADC1 的 DMA 句柄 */
static uint16_t           s_dma_buf[AUDIO_DMA_BUF_LEN];   /* 循环采样缓冲 */
static uint16_t           s_capture[AUDIO_CAPTURE_LEN];   /* 采集窗口 */
static volatile bool      s_running = false;

extern ADC_HandleTypeDef  hadc1;               /* CubeMX 生成的（adc.c） */

/* =============================================================================
 *  TIM2 配置：8kHz 触发源（用 **CH2 的比较输出** 作为 ADC 触发）
 * =============================================================================
 *  ⚠ 这里有个 F1 特有的坑，必须说清楚（否则采不到任何数据）：
 *
 *  F1 的 ADC 外部触发源**和 F4 不一样**，HAL 里可用的只有这几个：
 *      ADC_EXTERNALTRIGCONV_T1_CC1/CC2/CC3
 *      ADC_EXTERNALTRIGCONV_T2_CC2 / T2_CC3
 *      ADC_EXTERNALTRIGCONV_T3_CC1 / **T3_TRGO**
 *      ADC_EXTERNALTRIGCONV_T4_CC4 / T5_* / T8_*
 *  **没有 `T2_TRGO`**（那是 F4 的选项）。我第一次写的就是 T2_TRGO，
 *  被 arm-none-eabi-gcc 的语法检查直接报出来了 —— 这类错误如果只靠"看代码"
 *  很难发现，编译时才发现会浪费很多时间。
 *
 *  为什么选 **T2_CC2**（而不是 T3_TRGO）：
 *    · TIM3 正在驱动舵机（PB0 / TIM3_CH3，50Hz）。如果改用 TIM3_TRGO，
 *      就得把 TIM3 的 ARR 从 19999 改成 999（8kHz），**舵机会直接失控**。
 *      ── 一个定时器的 ARR 只有一套：要么服务 50Hz 舵机，要么服务 8kHz 触发。
 *    · TIM2 在本项目里**完全空闲**，用它最干净。
 *    · 用 **CC2 比较输出**（通道 2）而不是 TRGO：CC 触发点更直接，
 *      而且不依赖 TRGO 的复用配置，少一个出错环节。
 *
 *  定时参数（72MHz 系统时钟）：
 *      Prescaler = 8   → 72MHz / 9   = 8 MHz
 *      Period    = 999 → 8MHz / 1000 = **8 kHz**
 *      Pulse     = 500 → 占空比 50%（触发只用上升沿，占空比不影响频率）
 * ========================================================================== */
static void tim2_trigger_init(void)
{
    TIM_HandleTypeDef htim2;
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 9U - 1U;          /* 72MHz / 9  = 8MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 1000U - 1U;       /* 8MHz / 1000 = 8kHz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    /* 用 CH2 的 PWM 输出充当"周期性触发脉冲"。
     * 对 ADC 来说，CC2 的**上升沿**就是"开始转换"的命令。 */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 500U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    /* htim2 是局部变量：HAL 内部只在 Start 期间用它，之后寄存器已配好，
     * PWM 由硬件继续产生，所以这里可以安全返回。 */
}

/* =============================================================================
 *  DMA1_Channel1 配置：ADC1 → 内存，循环模式
 * ========================================================================== */
static void adc_dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_adc.Instance                 = DMA1_Channel1;
    s_hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;   /* ADC 数据寄存器地址不变 */
    s_hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;    /* 内存地址递增 */
    s_hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* 16 位 */
    s_hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;  /* 16 位 */
    s_hdma_adc.Init.Mode                = DMA_CIRCULAR;       /* ★ 循环：永不停止 */
    s_hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&s_hdma_adc) != HAL_OK) {
        Error_Handler();
    }

    /* 把 DMA 句柄挂到 ADC 上（HAL_ADC_Start_DMA 内部靠它找 DMA 配置） */
    __HAL_LINKDMA(&hadc1, DMA_Handle, s_hdma_adc);

    /* ⚠ 故意**不使能** DMA1_Channel1 中断。
     *   原因：8kHz 采样下 DMA 传输完成中断会以很高的频率打断 CPU，
     *   而我们**根本不需要它** —— 采样和搬运全由硬件自动完成，
     *   CPU 只在需要时主动读 DMA 的计数寄存器（轮询式取用）。
     *   这也是"用 DMA 把 CPU 解放出来"的正确用法：
     *   如果还要靠中断通知，就等于把 CPU 又拖回来了。 */
}

/* =============================================================================
 *  ADC1 重新配置：TIM2_TRGO 触发 + DMA
 *  ⚠ 这里直接改 hadc1 的 Init 字段并重新 HAL_ADC_Init，
 *    覆盖 CubeMX 生成的"软件触发单次转换"配置。
 * ========================================================================== */
static void adc_trigger_reconfig(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;   /* 只采 1 个通道 */
    hadc1.Init.ContinuousConvMode    = DISABLE;            /* 靠定时器触发，不连续 */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    /* ⚠ F1 的 ADC_InitTypeDef **没有** ExternalTrigConvEdge 成员（那是 F4 才有的）。
     *   F1 的"上升沿使能"是**编码在 ExternalTrigConv 的宏值里**的：
     *   ADC_EXTERNALTRIGCONV_T2_CC2 这个值本身就隐含了 EXTTRIG 位，
     *   再单独写 Edge 字段会编译报错（'ADC_InitTypeDef' has no member ...）。*/
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_CC2;  /* TIM2_CH2 上升沿触发 */
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* 通道 1（PA1）：13.5 周期采样时间。
     * 为什么从 239.5 改成 13.5：
     *   原来 4ms 采一次，多花 19µs 无所谓；现在每 125µs 要触发一次，
     *   而转换时间必须明显小于 125µs 才来得及。
     *   13.5 + 12.5 = 26 周期 @12MHz = 2.2µs，余量充足。
     *   MAX9814 是低阻抗运放输出，1.17µs 的采样窗口足够充电。 */
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* =============================================================================
 *  对外接口
 * ========================================================================== */
bool audio_init(void)
{
    tim2_trigger_init();
    adc_dma_init();
    adc_trigger_reconfig();

    /* 启动 ADC + DMA：HAL_ADC_Start_DMA 内部会开 ADC 的 DMA 请求位，
     * 之后每个 TIM2 触发都会自动产生一次转换 + 一次 DMA 搬运。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma_buf,
                          AUDIO_DMA_BUF_LEN) != HAL_OK) {
        Error_Handler();
    }

    /* 先采一小段让 DMA 填充，避免缓冲区里全是 0 */
    HAL_Delay(20);

    s_running = true;
    return true;
}

bool audio_is_running(void)
{
    return s_running;
}

uint16_t audio_dma_pos(void)
{
    /* 剩余未搬运的个数 → 当前位置 */
    const uint32_t remain = __HAL_DMA_GET_COUNTER(&s_hdma_adc);
    return (uint16_t)(AUDIO_DMA_BUF_LEN - remain);
}

uint16_t audio_peek(uint16_t pos)
{
    return s_dma_buf[pos & AUDIO_DMA_BUF_MASK];
}

/* 等"硬件又采了 n 个新样本"。
 *
 * ★ 为什么要等而不是延时 ★
 *   采样率是 TIM2 硬件产生的 8kHz（125µs 一个点），精度由晶振保证。
 *   如果用 HAL_Delay(1) 之类去"凑"时间，会引入 1ms 的 tick 抖动
 *   （相当于采样率误差可达 ±100%），对后续算频率是致命的。
 *   所以这里**只等 DMA 计数变化**，不看系统 tick。
 *
 * timeout_ms 只是防死循环（万一 ADC/DMA 没启动成功）。 */
static void wait_samples(uint32_t n, uint32_t timeout_ms)
{
    const uint16_t start = audio_dma_pos();
    const uint32_t t0    = HAL_GetTick();

    while (1) {
        const uint16_t now = audio_dma_pos();
        /* 环形增量：加 BUF_LEN 再取模，避免 uint16 回绕出错 */
        if ((uint32_t)((now - start) & AUDIO_DMA_BUF_MASK) >= n) {
            return;                       /* 新样本够了 */
        }
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return;                       /* 软超时：采样可能没在跑 */
        }
    }
}

uint16_t audio_measure_baseline(void)
{
    /* 取 AUDIO_TRIG_BASELINE_MS 毫秒的均值作为静音基线。
     * 样本直接从 DMA 滚动缓冲里按序取（间隔 8000/1000 = 8 个样本 = 1ms），
     * 不依赖 HAL_Delay —— 采样点的时间精度由硬件定时器保证。 */
    const uint32_t step = AUDIO_SAMPLE_RATE_HZ / 1000U;   /* 1ms 对应的样本数 */
    const uint32_t n    = AUDIO_TRIG_BASELINE_MS;         /* 取多少个点（1 点/ms）*/
    uint32_t sum = 0;
    uint16_t pos = audio_dma_pos();

    for (uint32_t i = 0; i < n; i++) {
        sum += s_dma_buf[pos & AUDIO_DMA_BUF_MASK];
        pos  = (uint16_t)((pos + step) & AUDIO_DMA_BUF_MASK);
        /* 等这 1ms 的新样本被硬件采完（最多等 2ms 防死循环） */
        wait_samples(step, 2U);
    }
    return (uint16_t)(sum / n);
}

bool audio_wait_trigger(uint16_t baseline)
{
    const uint32_t t0   = HAL_GetTick();
    uint32_t       hits = 0;

    while ((HAL_GetTick() - t0) < AUDIO_TRIG_TIMEOUT_MS) {
        /* 等一个新样本（硬件采满 8 个 = 1ms），保证检查是按采样节拍进行的 */
        wait_samples(8U, 2U);

        const uint16_t s = s_dma_buf[audio_dma_pos() & AUDIO_DMA_BUF_MASK];
        /* 与基线的绝对偏差（信号有正有负，所以取绝对值）*/
        const uint16_t d = (s > baseline) ? (uint16_t)(s - baseline)
                                          : (uint16_t)(baseline - s);

        if (d >= AUDIO_TRIG_DELTA) {
            hits++;
            if (hits >= AUDIO_TRIG_HITS_N) {
                return true;      /* 连续超阈值 → 判定为有效语音 */
            }
        } else {
            hits = 0;             /* 中途掉下来就重新计数（防单次脉冲误触发）*/
        }
    }
    return false;                 /* 超时：没有有效语音 */
}

const uint16_t *audio_capture_window(void)
{
    /* 采集 AUDIO_CAPTURE_LEN（8000 = 1 秒）个样本。
     *
     * ★ 核心思路：**不靠软件延时凑采样率，只靠硬件定时器** ★
     *   · DMA 在 16384 样本的滚动缓冲里循环写（采样由 TIM2 精确控制为 8kHz）
     *   · 每等"正好 1024 个新样本就绪"（128ms），就把这 1024 个按序拷出来
     *   · 用 DMA 的计数寄存器（CNDTR）判断"新增了多少"，
     *     **不看系统 tick**（tick 只有 1ms 分辨率，对 125µs 太粗）
     *
     * 因为每块的"起步时刻"是连续的（上一块结束时 DMA 已前进到下一块开头），
     * 所以拼出来的 8000 点时间轴是严格等间隔的 1 秒音频。 */
    uint32_t produced = 0;
    uint16_t last     = audio_dma_pos();

    while (produced < AUDIO_CAPTURE_LEN) {
        const uint16_t now = audio_dma_pos();
        /* 环形增量（加 MASK+1 再取模，避免 uint16 回绕出负数）*/
        uint32_t inc = (uint32_t)((now - last) & AUDIO_DMA_BUF_MASK);

        while (inc > 0 && produced < AUDIO_CAPTURE_LEN) {
            s_capture[produced++] = s_dma_buf[last & AUDIO_DMA_BUF_MASK];
            last = (uint16_t)((last + 1U) & AUDIO_DMA_BUF_MASK);
            inc--;
        }
        /* DMA 没前进时不要死等（万一采样没启动），加一个软超时 */
        if (produced < AUDIO_CAPTURE_LEN) {
            uint32_t guard = 0;
            while (audio_dma_pos() == now && guard++ < 2000000U) { /* 自旋等新样本 */ }
            if (guard >= 2000000U) {
                break;            /* 采样疑似停了：返回已采到的部分 */
            }
        }
    }
    return s_capture;
}

uint32_t audio_window_len(void)
{
    return AUDIO_CAPTURE_LEN;
}

void audio_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    s_running = false;
}
