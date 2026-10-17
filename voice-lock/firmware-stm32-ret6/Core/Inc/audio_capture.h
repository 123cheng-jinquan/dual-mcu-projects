/* =============================================================================
 *  audio_capture.h — 8kHz 音频采集（TIM2 触发 ADC1 + DMA 循环）
 * =============================================================================
 *  为什么需要这个模块（这是整个项目升级的核心）
 *  ---------------------------------------------------------------------------
 *  原毕设用的是 **单次转换 + 软件延时**：
 *        Mic_ReadOnce()  { HAL_ADC_Start(); HAL_ADC_PollForConversion(); }
 *        MeasureBaseline() { ... HAL_Delay(2); }
 *    推出来的采样率只有 **250 Hz**（SAMPLE_INTERVAL_MS = 4）。
 *
 *  250 Hz 意味着**奈奎斯特极限只有 125 Hz**，而人说话的基频是
 *     男声 85~180 Hz、女声 165~255 Hz
 *  ⇒ **采到的数据里连人声基频都没有**，只剩"音量包络"。
 *    这也正是原论文里要写"云端声纹仅作接口验证"的根本原因：
 *    **数据本身不支持声纹识别**，不是软件写法的问题。
 *
 *  本模块把采样率提到 **8 kHz**（32 倍），做法是：
 *    · TIM2 以 8 kHz 产生 Update 事件 → 作为 ADC 的外部触发源（TRGO）
 *    · ADC1 每次触发转换 1 个通道，结果由 **DMA1_Channel1 循环搬运**到内存
 *    · CPU **完全不参与采样**（不轮询、不进中断），可以去做别的事
 *
 *  为什么不用"ADC 连续转换"：
 *    连续转换模式下转换间隔由 ADC 时钟决定，**不能精确控制成 8 kHz**
 *    （而且 F1 的 ADC 时钟最高 14MHz，很难凑出整数）。用定时器触发才能
 *    得到**精确且可调**的采样率 —— 采样率不准会直接让频率特征算错。
 *
 *  频率计算（72MHz 系统时钟）
 *  ---------------------------------------------------------------------------
 *    TIM2:  Prescaler = 8  → 72MHz / 9  = 8 MHz 计数频率
 *           Period    = 999 → 8MHz / 1000 = **8 kHz** 触发频率
 *           Pulse     = 500 → 50% 占空比（占空比对触发没影响，给个中间值）
 *           TRGO      = Update 事件
 *    ADC1:  预分频 ÷6 → 12 MHz ADC 时钟
 *           采样时间 13.5 周期 → 13.5 + 12.5 = 26 周期 = 2.2 µs < 125 µs ✅
 *
 *  内存占用（RET6 有 64KB RAM，够用；C8T6 只有 20KB，放不下）
 *  ---------------------------------------------------------------------------
 *    DMA 循环缓冲  AUDIO_DMA_BUF_LEN 个 uint16_t
 *    采集窗口      AUDIO_CAPTURE_SEC  秒 × 8000
 * ========================================================================== */

#ifndef __AUDIO_CAPTURE_H
#define __AUDIO_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 采样参数 ---------------------------------------------------------- */
#define AUDIO_SAMPLE_RATE_HZ     8000U    /* 8 kHz：覆盖完整语音频段 */
#define AUDIO_ADC_MAX            4095U    /* 12 位 ADC */
#define AUDIO_ADC_MID            2048U    /* 理论上 MAX9814 输出中心 = VDD/2 */

/* DMA 循环缓冲长度。
 *
 * ★ 为什么开到 16384（而不是几百）★
 *   采样时机必须完全由**硬件定时器**决定，不能靠软件延时来"凑"采样率。
 *   做法：DMA 在 16K 样本（约 2 秒 @8kHz）的缓冲里循环写，缓冲本身
 *   就是"最近 2 秒的连续音频"。采集时只要等定时器把 **1024 个样本
 *   （128ms）写完**（用 DMA 计数寄存器精确判断，不看系统 tick），
 *   就能确定"恰好 128ms 的新样本已经就绪"，然后按序拷贝出来。
 *   ⇒ CPU 只在"整块就绪"时搬运，**采样间隔的精度由硬件保证**。
 *
 * RAM 代价：16384 × 2 = 32KB。RET6 有 64KB，够用；
 *          （C8T6 只有 20KB，这正是必须换 RET6 的原因之一。）
 * 必须是 2 的幂（用位与取模，比除法快）。 */
#define AUDIO_DMA_BUF_LEN        16384U
#define AUDIO_DMA_BUF_MASK       (AUDIO_DMA_BUF_LEN - 1U)

/* 单次采集窗口长度（样本数）。
 * 1 秒 @8kHz = 8000 个样本。为什么 1 秒：声纹/关键词识别通常需要
 * 0.8~2 秒的语音，短窗信息量不足。RAM：8000 × 2 = 16KB。 */
#define AUDIO_CAPTURE_SEC        1U
#define AUDIO_CAPTURE_LEN        (AUDIO_SAMPLE_RATE_HZ * AUDIO_CAPTURE_SEC)

/* ---- 触发检测参数（★ 基于"短时能量 RMS"，不是单样本阈值）---------------
 * ⚠ 这里踩过一个坑，记下来：
 *   最初我按原毕设的思路写"连续 N 个样本偏离基线就触发"。在 250Hz 时
 *   N=3 是成立的（12ms 才 3 个样本）；但采样率提到 8kHz 后，
 *   **语音波形是围绕基线上下振荡的，每半个周期就穿越基线一次**
 *   （低频语音约每 1~3ms 一次 = 每 8~24 个样本），
 *   所以"连续 96 个样本都偏离基线"几乎不可能出现 → 永远不触发。
 *   ⇒ 表面现象是"一直 timeout"。
 *
 * 正确做法（也是业界标准 VAD 的做法）：
 *   把音频分成 10ms 的**帧**，算每帧的 **RMS 能量**，
 *   能量超过"噪声底 × 倍数"就认为有语音，**连续若干帧**超阈才算触发。
 *   这样对波形振荡完全免疫，判断的是"响度"而不是"瞬时电平"。
 * ------------------------------------------------------------------------- */
#define AUDIO_FRAME_MS           10U      /* 每帧 10ms */
#define AUDIO_FRAME_SAMPLES      ((AUDIO_SAMPLE_RATE_HZ / 1000U) * AUDIO_FRAME_MS)  /* 80 点 */
#define AUDIO_TRIG_BASELINE_MS   200U     /* 基线估计窗口（同时用于测噪声底）*/
#define AUDIO_TRIG_MINSTAT_FRAMES 10U     /* 用 10 帧的最小值估噪声底（抗污染）*/
#define AUDIO_TRIG_WIN_N         4U       /* 用最近 4 帧的**峰值**判触发 */
#define AUDIO_TRIG_MULT_NUM      3U       /* 阈值 = 噪声底 × 1.5（分子）*/
#define AUDIO_TRIG_MULT_DEN      2U       /*                      （分母）*/
#define AUDIO_TRIG_MIN_RMS       150U     /* 绝对下限。
                                           * 为什么要有：噪声底测出来太小（或测错）
                                           * 时，阈值不能低于 150，否则环境波动就误触发。
                                           * 实测说话 RMS 约 800~1100，150 留足余量。 */
#define AUDIO_TRIG_TIMEOUT_MS    5000U    /* 等待语音触发的超时 */

/* ---- API --------------------------------------------------------------- */

/** 初始化结果码（用于精确定位失败在哪一步）
 *  为什么要这个：原来失败就直接调 Error_Handler()（关中断 + 死循环），
 *  板子会"一声不响地卡住"，串口什么都看不到，根本无法定位。
 *  改成返回状态码后，调用方可以一步一步打印出卡在哪。 */
typedef enum {
    AUDIO_INIT_OK         = 0,
    AUDIO_INIT_TIM2_FAIL  = 1,   /* TIM2 PWM 初始化失败 */
    AUDIO_INIT_DMA_FAIL   = 2,   /* DMA1_Channel1 初始化失败 */
    AUDIO_INIT_ADC_FAIL   = 3,   /* ADC1 重新配置（HAL_ADC_Init）失败 */
    AUDIO_INIT_CH_FAIL    = 4,   /* ADC 通道配置（HAL_ADC_ConfigChannel）失败 */
    AUDIO_INIT_START_FAIL = 5,   /* HAL_ADC_Start_DMA 失败 */
} audio_init_rc_t;

/** 初始化：配置 TIM2 触发源、DMA1，把 ADC1 从"软件触发单次转换"
 *  改成"TIM2_CH2 触发 + DMA 循环"。
 *  返回 AUDIO_INIT_OK = 成功；其它值表示失败在哪一步。
 *
 *  ⚠ 调用方必须写成 `if (audio_init() == AUDIO_INIT_OK)`，
 *    **不能**写 `if (audio_init())` —— 因为 AUDIO_INIT_OK == 0，
 *    而 C 里"0 视为假"，那样会把成功判成失败。
 *
 *  ⚠ 必须在 MX_ADC1_Init() 之后调用（会覆盖它的触发源配置）。 */
audio_init_rc_t audio_init(void);

/** 采样是否在跑（调试用） */
bool audio_is_running(void);

/** 取 DMA 当前的写入位置（调试用：它应该一直在变，说明采样在跑） */
uint16_t audio_dma_pos(void);

/** 按位置读一个样本（调试/自检用）。
 *  ⚠ 这是"回看历史"的接口：pos 是 DMA 缓冲区内任意位置，
 *    调用方自己负责推进 pos（一般每次 += 8000/1000 = 8，即前进 1ms）。
 *    返回的是 0~4095 的原始 ADC 值。 */
uint16_t audio_peek(uint16_t pos);

/** 估计环境噪声基线（返回 ADC 均值，用于观察偏置）。 */
uint16_t audio_measure_baseline(void);

/** 估计环境"噪声底"的 RMS 幅度（用于观察/排查）。
 *  返回的是交流分量的 RMS（已去掉直流偏置），单位 ADC counts。
 *  安静时通常 10~100；有声音时几百~上千。 */
uint16_t audio_measure_noise_rms(void);

/** 在 ms 毫秒内，**逐帧扫描并返回最大的那一帧 RMS**。
 *  用途：标定"说话时到底有多响" —— 因为语音是起伏的，
 *        取平均会被静音段拉低，取峰值才反映真实响度。
 *  ⚠ 这个函数会阻塞 ms 毫秒。 */
uint16_t audio_measure_peak_rms(uint32_t ms);

/** 等待语音触发（基于短时能量 RMS 的 VAD）。
 *  内部自己测噪声底并算阈值，不需要参数。
 *  返回 true = 检测到有效语音。 */
bool audio_wait_trigger(void);

/** 上一次触发检测的诊断值（**触发超时时**用来判断原因）。
 *  ⚠ 只在 audio_wait_trigger() 返回 false 后被更新。 */
typedef struct {
    uint16_t noise_rms;    /* 当时估到的噪声底 */
    uint16_t thresh_rms;   /* 当时用的触发阈值 */
    uint16_t peak_rms;     /* 等待期间实际见到的最大 RMS */
} audio_trig_diag_t;

void audio_get_trigger_diag(audio_trig_diag_t *out);

/** 采集 AUDIO_CAPTURE_LEN 个样本到内部缓冲区（阻塞，约 1 秒）。
 *  返回指向缓冲区的指针（只读，长度 = AUDIO_CAPTURE_LEN）。 */
const uint16_t *audio_capture_window(void);

/** 采集缓冲区的长度（样本数） */
uint32_t audio_window_len(void);

/** 停止采样（低功耗时用） */
void audio_stop(void);

#endif /* __AUDIO_CAPTURE_H */
