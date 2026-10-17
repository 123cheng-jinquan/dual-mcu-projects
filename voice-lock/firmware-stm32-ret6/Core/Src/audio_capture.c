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
static audio_init_rc_t    s_last_rc = AUDIO_INIT_OK;      /* 上次初始化结果 */

/* 上一次触发检测的诊断值（触发超时时用于排查"是没说话还是阈值太高"）*/
static uint16_t           s_last_trigger_noise  = 0;   /* 当时的噪声底 RMS */
static uint16_t           s_last_trigger_thresh = 0;   /* 当时用的阈值 RMS */
static uint16_t           s_last_trigger_peak   = 0;   /* 实际见到的最大 RMS */

extern ADC_HandleTypeDef  hadc1;               /* CubeMX 生成的（adc.c） */

/* =============================================================================
 *  整数开方（牛顿迭代）
 * =============================================================================
 *  为什么不用 math.h 的 sqrtf：
 *    Cortex-M3 没有 FPU，浮点开方要走软件库（几微秒 + 拉进浮点库代码）。
 *    这里要开方的都是 32 位整数（能量值），用整数牛顿迭代几轮就收敛，
 *    更快而且不带浮点依赖。 */
static uint32_t isqrt_u32(uint32_t v)
{
    if (v == 0U) { return 0U; }
    uint32_t x = v;
    uint32_t y = (x + 1U) / 2U;
    while (y < x) {
        x = y;
        y = (x + v / x) / 2U;
    }
    return x;
}

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
static audio_init_rc_t tim2_trigger_init(void)
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
        return AUDIO_INIT_TIM2_FAIL;
    }

    /* 用 CH2 的 PWM 输出充当"周期性触发脉冲"。
     * 对 ADC 来说，CC2 的**上升沿**就是"开始转换"的命令。 */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 500U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        return AUDIO_INIT_TIM2_FAIL;
    }

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    return AUDIO_INIT_OK;
    /* htim2 是局部变量：HAL 内部只在 Start 期间用它，之后寄存器已配好，
     * PWM 由硬件继续产生，所以这里可以安全返回。 */
}

/* =============================================================================
 *  DMA1_Channel1 配置：ADC1 → 内存，循环模式
 * ========================================================================== */
static audio_init_rc_t adc_dma_init(void)
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
        return AUDIO_INIT_DMA_FAIL;
    }

    /* 把 DMA 句柄挂到 ADC 上（HAL_ADC_Start_DMA 内部靠它找 DMA 配置） */
    __HAL_LINKDMA(&hadc1, DMA_Handle, s_hdma_adc);

    return AUDIO_INIT_OK;

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
static audio_init_rc_t adc_trigger_reconfig(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* ★★ 必须先 DeInit，否则 HAL_ADC_Init() 会失败 ★★
     * ---------------------------------------------------------------------
     * CubeMX 生成的 MX_ADC1_Init() 已经调用过一次 HAL_ADC_Init()，
     * 此时 hadc1.State 是 READY。HAL 的状态机不允许在非 RESET 状态下
     * 再次 HAL_ADC_Init() —— 会直接返回 HAL_ERROR（或者更糟：半配置状态）。
     *
     * 所以重新配置前必须：
     *   HAL_ADC_DeInit()  →  State 回到 RESET + 关 ADC 时钟
     *   HAL_ADC_Init()    →  用新参数重新初始化（会再次调 MspInit 开时钟）
     *
     * 这也是"为什么板子一声不响地卡住"的直接原因：
     *   原代码在这一步返回失败 → 调 Error_Handler() → 关中断 + 死循环。
     * ------------------------------------------------------------------- */
    if (HAL_ADC_DeInit(&hadc1) != HAL_OK) {
        return AUDIO_INIT_ADC_FAIL;
    }

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
        return AUDIO_INIT_ADC_FAIL;
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
        return AUDIO_INIT_CH_FAIL;
    }
    return AUDIO_INIT_OK;
}

/* =============================================================================
 *  对外接口
 * ========================================================================== */
audio_init_rc_t audio_init(void)
{
    /* ⚠ 返回枚举而不是 bool：
     *   原来失败直接 Error_Handler()（关中断 + 死循环）→ 板子一声不响地卡住，
     *   串口什么都看不到。改成返回状态码后，调用方能打印出"卡在哪一步"。
     *   下面的 return 必须返回 audio_init_rc_t 的枚举值，不能写 true/false
     *   （Keil ARMCC 会报 warning #188-D "enumerated type mixed with another type"）。*/
    audio_init_rc_t rc;

    rc = tim2_trigger_init();
    if (rc != AUDIO_INIT_OK) { s_last_rc = rc; return rc; }

    rc = adc_dma_init();
    if (rc != AUDIO_INIT_OK) { s_last_rc = rc; return rc; }

    rc = adc_trigger_reconfig();
    if (rc != AUDIO_INIT_OK) { s_last_rc = rc; return rc; }

    /* 启动 ADC + DMA：HAL_ADC_Start_DMA 内部会开 ADC 的 DMA 请求位，
     * 之后每个 TIM2 触发都会自动产生一次转换 + 一次 DMA 搬运。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma_buf,
                          AUDIO_DMA_BUF_LEN) != HAL_OK) {
        s_last_rc = AUDIO_INIT_START_FAIL;
        return AUDIO_INIT_START_FAIL;
    }

    /* 先采一小段让 DMA 填充，避免缓冲区里全是 0 */
    HAL_Delay(20);

    s_running = true;
    s_last_rc = AUDIO_INIT_OK;
    return AUDIO_INIT_OK;
}

audio_init_rc_t audio_last_rc(void)
{
    return s_last_rc;
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

/* =============================================================================
 *  取一帧并计算它的 RMS 交流幅度
 * =============================================================================
 *  为什么用 RMS 而不是"单样本阈值"：
 *    语音波形是围绕直流偏置上下振荡的，每半个周期就穿越基线一次。
 *    如果按"单个样本偏离基线多少"判断，振荡本身的过零点会让计数器不断归零
 *    （原毕设 250Hz 时 12ms 只有 3 个点还勉强能用，提到 8kHz 后彻底失效）。
 *    RMS 衡量的是"这一小段有多响"，对振荡相位完全不敏感 ⇒ 稳定可靠。
 *
 *  实现要点：
 *    · 先减掉直流均值（否则 RMS 会被 1500 左右的偏置完全淹没）
 *    · 用 int32 累加平方；AUDIO_FRAME_SAMPLES=80、幅度最大 4095 时
 *      平方和最大 80 × 4095² ≈ 1.34e9，在 int32（2.1e9）范围内，安全
 *    · 不做开方（省时间）：直接比较"均方值"和"阈值的平方"等价
 * ========================================================================== */
static uint32_t frame_energy(uint16_t pos, uint16_t mean)
{
    int32_t acc = 0;

    for (uint32_t i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
        const int32_t v = (int32_t)s_dma_buf[(pos + i) & AUDIO_DMA_BUF_MASK]
                          - (int32_t)mean;
        acc += v * v;
    }
    return (uint32_t)(acc / (int32_t)AUDIO_FRAME_SAMPLES);   /* 均方值 */
}

uint16_t audio_measure_baseline(void)
{
    /* 取 AUDIO_TRIG_BASELINE_MS 毫秒的均值作为直流基线。
     * 样本按 1ms 间隔从 DMA 滚动缓冲里取，不依赖 HAL_Delay ——
     * 采样时刻的精度由硬件定时器保证。 */
    const uint32_t step = AUDIO_SAMPLE_RATE_HZ / 1000U;   /* 1ms 对应的样本数 */
    const uint32_t n    = AUDIO_TRIG_BASELINE_MS;

    /* ⚠ 为什么要重试：第一次调用时（比如刚上电、DMA 才启动 20ms），
     *   滚动缓冲里可能还有大段未填充的区域（全 0），算出来的均值会是 0。
     *   均值 0 会让后续"去掉直流"完全失效 → 特征全错。
     *   所以这里最多重测 3 次，取到非 0 为止。 */
    for (uint32_t attempt = 0; attempt < 3U; attempt++) {
        uint32_t sum = 0;
        uint16_t pos = audio_dma_pos();

        for (uint32_t i = 0; i < n; i++) {
            sum += s_dma_buf[pos & AUDIO_DMA_BUF_MASK];
            pos  = (uint16_t)((pos + step) & AUDIO_DMA_BUF_MASK);
            wait_samples(step, 2U);  /* 等新样本被采完（软超时防死循环）*/
        }

        const uint16_t mean = (uint16_t)(sum / n);
        if (mean != 0U) {
            return mean;
        }
        /* mean==0：可能缓冲还没填满，等 50ms 再来一次 */
        HAL_Delay(50);
    }
    return (uint16_t)AUDIO_ADC_MID;   /* 兜底：用理论中点，避免返回 0 */
}

uint16_t audio_measure_noise_rms(void)
{
    /* =========================================================================
     * 用"最小值统计"估噪声底 —— 而不是取平均
     * =========================================================================
     *  为什么不用平均（踩过的坑）：
     *    原来取 20 帧的**平均**当作噪声底。但如果这段窗口里用户已经开始
     *    说话了（或者有人在旁边讲话），平均值就被"污染"得很高。
     *    实测：同一环境下两次测量得到 104 和 273（差 2.6 倍）——
     *    阈值因此被抬到 819，导致**永远不触发**。
     *
     *  为什么最小值可靠：
     *    语音是断断续续的（音节之间有停顿），一段 100ms 里总会有几帧
     *    几乎是纯噪声。取**最小**的那一帧就接近真实噪声底，
     *    语音的存在只会让某些帧变大，**不可能让最小值也变大**。
     *    这就是"最小值统计（minimum statistics）"噪声估计的核心思想。
     *
     *  代价：需要在一帧 RMS 里包含"真正安静"的时刻。对门禁场景
     *       （用户对着门说话）完全成立。
     * ======================================================================= */
    uint32_t min_ms2 = 0xFFFFFFFFU;
    const uint16_t mean = audio_measure_baseline();

    for (uint32_t i = 0; i < AUDIO_TRIG_MINSTAT_FRAMES; i++) {
        const uint16_t pos = audio_dma_pos();
        const uint32_t ms2 = frame_energy(pos, mean);
        if (ms2 < min_ms2) {
            min_ms2 = ms2;
        }
        wait_samples(AUDIO_FRAME_SAMPLES, 20U);
    }
    if (min_ms2 == 0xFFFFFFFFU) { min_ms2 = 0U; }

    /* 整数开方（牛顿迭代）：比调 math.h 的 sqrtf 省去浮点库依赖，
     * 在没有 FPU 的 Cortex-M3 上也更快。 */
    return (uint16_t)isqrt_u32(min_ms2);
}

uint16_t audio_measure_peak_rms(uint32_t ms)
{
    /* 逐帧扫描，返回最大帧的 RMS。
     * 为什么取"峰值"而不是"平均"：
     *   语音是起伏的（音节之间有停顿），如果在一段话里取平均，
     *   静音段会把结果拉低很多，量出来偏小 ⇒ 没法用来定阈值。
     *   取峰值才能反映"说话时最响能达到多少"。 */
    const uint32_t frames = ms / AUDIO_FRAME_MS;
    const uint16_t mean   = audio_measure_baseline();
    uint32_t peak_ms2     = 0;

    for (uint32_t i = 0; i < frames; i++) {
        const uint16_t pos = audio_dma_pos();
        const uint32_t ms2 = frame_energy(pos, mean);
        if (ms2 > peak_ms2) {
            peak_ms2 = ms2;
        }
        wait_samples(AUDIO_FRAME_SAMPLES, 20U);
    }

    return (uint16_t)isqrt_u32(peak_ms2);
}

bool audio_wait_trigger(void)
{
    /* ★ 先测当前环境的噪声底，再据此定阈值 ★
     * 噪声底用"最小值统计"估（见 audio_measure_noise_rms 的说明），
     * 所以**即使测量期间用户已经在说话也不会被污染**。
     *
     * 阈值 = 噪声底 × 1.5，并有绝对下限 150。
     * 为什么倍数只取 1.5：实测噪声底在 176~313 波动，而说话 RMS 是 800~1100，
     * 取 1.5 倍（≈260~470）稳稳落在两者之间；取 2~3 倍反而会高过轻声说话。
     * 绝对下限 150 是为了防止"噪声底测出来很小"时阈值过低、环境一波动就误触发。 */
    const uint16_t noise_rms = audio_measure_noise_rms();

    uint32_t thresh_rms = ((uint32_t)noise_rms * AUDIO_TRIG_MULT_NUM)
                          / AUDIO_TRIG_MULT_DEN;
    if (thresh_rms < AUDIO_TRIG_MIN_RMS) {
        thresh_rms = AUDIO_TRIG_MIN_RMS;
    }
    const uint32_t thresh_ms = thresh_rms * thresh_rms;   /* 和均方值比较，省开方 */

    const uint32_t t0     = HAL_GetTick();
    uint32_t       win[AUDIO_TRIG_WIN_N] = {0};   /* 最近 N 帧的均方值 */
    uint32_t       idx    = 0;
    uint32_t       peak_max = 0;                  /* 记录见过的最大能量（排查用）*/

    while ((HAL_GetTick() - t0) < AUDIO_TRIG_TIMEOUT_MS) {
        /* 取一帧（10ms = 80 个样本），算它的能量 */
        const uint16_t mean = AUDIO_ADC_MID;   /* 用中点近似直流，省一次遍历 */
        const uint16_t pos  = audio_dma_pos();
        const uint32_t ms2  = frame_energy(pos, mean);

        /* ★ 用"最近 N 帧里的最大值"判触发，而不是"连续 N 帧都超阈" ★
         *   为什么：语音能量是起伏的（一个音节里也有强弱变化），
         *   "连续都超阈"很容易被其中一帧的低谷打断。
         *   取窗口峰值则只要 40ms 内出现过一次足够响的能量就算，
         *   对"说话起音"的捕捉灵敏得多。 */
        win[idx] = ms2;
        idx = (idx + 1U) % AUDIO_TRIG_WIN_N;

        uint32_t peak = 0;
        for (uint32_t i = 0; i < AUDIO_TRIG_WIN_N; i++) {
            if (win[i] > peak) { peak = win[i]; }
        }
        if (peak > peak_max) { peak_max = peak; }

        if (peak >= thresh_ms) {
            return true;
        }

        /* 等这一帧的数据被硬件采完，保证下一帧不重叠 */
        wait_samples(AUDIO_FRAME_SAMPLES, 20U);
    }

    /* 超时也把"见过的最大能量"记下来（换算成 RMS 更好读），
     * 便于排查"到底是没说话还是阈值定太高"。 */
    s_last_trigger_noise  = noise_rms;
    s_last_trigger_thresh = (uint16_t)thresh_rms;
    s_last_trigger_peak   = (uint16_t)isqrt_u32(peak_max);
    return false;                 /* 超时：没有有效语音 */
}

void audio_get_trigger_diag(audio_trig_diag_t *out)
{
    if (out == 0) { return; }
    out->noise_rms  = s_last_trigger_noise;
    out->thresh_rms = s_last_trigger_thresh;
    out->peak_rms   = s_last_trigger_peak;
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
