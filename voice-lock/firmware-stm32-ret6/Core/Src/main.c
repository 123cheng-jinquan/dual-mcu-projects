/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "audio_capture.h"   /* 8kHz 音频采集（TIM2 触发 ADC1 + DMA）*/
#include "uart_proto.h"      /* 双片通信协议（F103RET6 ↔ ESP32-S3）*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>       /* toupper() —— 命令大小写归一化 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t mean;
    uint16_t min;
    uint16_t max;
    uint16_t p2p;
    uint32_t energy;
    uint32_t seg1;
    uint32_t seg2;
    uint32_t seg3;
    uint32_t seg4;
    uint32_t seg5;
    uint32_t seg6;
    uint32_t seg7;
    uint32_t seg8;
} VoiceFeature_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_MIN_US              500U
#define SERVO_MAX_US              2500U
#define DOOR_OPEN_ANGLE           0U
#define DOOR_CLOSE_ANGLE          90U

#define ESP_RX_BUF_LEN            512U

/* WiFi / Server 参数 */
#define WIFI_SSID                 "myk50"
#define WIFI_PASSWORD             "88888888"
#define SERVER_IP                 "192.168.54.204"
#define SERVER_PORT               5000U

/* =============================================================================
 *  采样参数 —— ⚠ 这些是【升级前】的值，保留作为对比证据
 * =============================================================================
 *  升级前：220 点 × 每 4ms 一个  = 等效采样率 **250 Hz**
 *          → 奈奎斯特极限只有 125Hz
 *          → 而人声基频是 85~255Hz
 *          ⇒ **采到的数据里连人声基频都没有**，只剩"音量包络"
 *          ⇒ 这就是原论文要写"云端声纹仅作接口验证"的根本原因
 *
 *  升级后：TIM2 硬件触发 ADC + DMA，**8000 Hz**（实测 7996~8000Hz）
 *          → 奈奎斯特极限 4000Hz，覆盖完整语音频段
 *          → 相关定义见 Core/Inc/audio_capture.h
 *
 *  ⚠ 下面这两个宏现在**不再被用于实际采集**（采集已移到 audio_capture.c），
 *    保留它们是为了让"250Hz → 8kHz"这个改动在代码里可见、可追溯。
 * ========================================================================== */
#define SAMPLE_COUNT              220U    /* 【历史】原 220 点 */
#define SAMPLE_INTERVAL_MS        4U      /* 【历史】原 4ms 间隔 → 250Hz */

/* 语音触发参数（同样已被 audio_capture.h 的 AUDIO_TRIG_* 取代，保留作对比）*/
#define TRIGGER_BASELINE_COUNT    24U
#define TRIGGER_DELTA             220U
#define TRIGGER_HITS              3U
#define TRIGGER_TIMEOUT_MS        2500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart1_rx_byte = 0;
uint8_t uart3_rx_byte = 0;

volatile uint8_t uart1_cmd_ready = 0;
volatile uint8_t uart1_cmd_byte = 0;

volatile uint16_t esp_rx_len = 0;
char esp_rx_buf[ESP_RX_BUF_LEN];

uint16_t mic_adc_value = 0;
uint8_t door_is_open = 0;
uint8_t wifi_ready = 0;
uint8_t server_connected = 0;

/* 连续通过计数：必须连续两次 PASS 才真正开锁 */
uint8_t pass_streak = 0;

char last_result[16] = "BOOT";

/* ★ OLED 演示用的判定信息 ★
 *   last_sim_permille  ：ESP32 回传的相似度（千分比，如 732 = 0.732）
 *   为什么要放到 OLED 上：门锁指示改成 OLED 之后，屏幕是唯一反馈渠道，
 *   显示"相似度 vs 阈值"能直观说明"为什么这次通过/不通过" —— 演示效果最好。 */
uint16_t last_sim_permille  = 0U;

uint32_t last_adc_tick = 0;
uint32_t last_oled_tick = 0;

/* ⚠ 原来的 adc_samples[220] 已删除 —— 升级后采集窗口（8000 点）由
 *   audio_capture.c 内部的静态缓冲区持有，ExtractVoiceFeature() 直接
 *   读那个指针，不再复制一份（省 16KB RAM）。
 *   见 CaptureVoiceWindow() 里的说明。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Debug_SendString(const char *str);
static void Debug_Printf(const char *fmt, ...);

static void ESP_ClearRxBuffer(void);
static void ESP_PrintRxBuffer(void);
static void ESP8266_HardwareReset(void);
static void ESP8266_SendString(const char *str);
static int  ESP_WaitForAny(uint32_t timeout_ms,
                           const char *succ1,
                           const char *succ2,
                           const char *succ3,
                           const char *fail1,
                           const char *fail2);
static uint8_t ESP_SendCmdWait(const char *cmd,
                               uint32_t timeout_ms,
                               const char *succ1,
                               const char *succ2,
                               const char *succ3);
static uint8_t ESP_InitWiFi(void);
static uint8_t ESP_ConnectServer(void);
static uint8_t ESP_CloudVerifyOnce(void);

static void SendAudioToESP32(void);   /* 8kHz 采 1 秒 → 压缩 → 发往 ESP32-S3 */

static void Door_Open(void);
static void Door_Close(void);

static void Audio_SelfTest(void);     /* 音频采集自检（'t' 命令） */
static void Audio_Calibrate(void);    /* 声学标定（'k' 命令） */
static void Uart3_Diag(void);         /* USART3 硬件诊断（'x' 命令） */
static void Uart3_LoopbackTest(void); /* USART3 回环测试（'L' 命令） */
static void Audio_VerifyAndAct(void); /* 完整门禁流程（'u' 命令） */

static uint16_t Mic_ReadOnce(void);
static uint16_t MeasureBaseline(void);
static uint8_t WaitForVoiceTrigger(void);
static void CaptureVoiceWindow(void);
static VoiceFeature_t ExtractVoiceFeature(void);

static void I2C_Diag(void);           /* OLED/I2C 总线诊断（'D' 命令）*/
static void OLED_UpdateStatus(void);
static void ShowHelp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Debug_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);
}

static void Debug_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);
}

static void ESP_ClearRxBuffer(void)
{
    __disable_irq();
    memset(esp_rx_buf, 0, sizeof(esp_rx_buf));
    esp_rx_len = 0;
    __enable_irq();
}

static void ESP_PrintRxBuffer(void)
{
    if (esp_rx_len > 0)
    {
        Debug_SendString("\r\n[ESP RX BEGIN]\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)esp_rx_buf, strlen(esp_rx_buf), 1000);
        Debug_SendString("\r\n[ESP RX END]\r\n");
    }
    else
    {
        Debug_SendString("\r\n[ESP RX EMPTY]\r\n");
    }
}

static void ESP8266_HardwareReset(void)
{
    HAL_GPIO_WritePin(ESP82660_EN_GPIO_Port, ESP82660_EN_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(ESP82660_RST_GPIO_Port, ESP82660_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(ESP82660_RST_GPIO_Port, ESP82660_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(1500);

    ESP_ClearRxBuffer();
    wifi_ready = 0;
    server_connected = 0;
    pass_streak = 0;
    strcpy(last_result, "RESET");
    OLED_UpdateStatus();
}

static void ESP8266_SendString(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
}

static int ESP_WaitForAny(uint32_t timeout_ms,
                          const char *succ1,
                          const char *succ2,
                          const char *succ3,
                          const char *fail1,
                          const char *fail2)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (succ1 && strstr(esp_rx_buf, succ1)) return 1;
        if (succ2 && strstr(esp_rx_buf, succ2)) return 1;
        if (succ3 && strstr(esp_rx_buf, succ3)) return 1;

        if (fail1 && strstr(esp_rx_buf, fail1)) return 0;
        if (fail2 && strstr(esp_rx_buf, fail2)) return 0;

        if (strstr(esp_rx_buf, "ERROR")) return 0;
        if (strstr(esp_rx_buf, "FAIL")) return 0;
        if (strstr(esp_rx_buf, "busy")) return 0;
        if (strstr(esp_rx_buf, "link is not valid")) return 0;
    }

    return -1;
}

static uint8_t ESP_SendCmdWait(const char *cmd,
                               uint32_t timeout_ms,
                               const char *succ1,
                               const char *succ2,
                               const char *succ3)
{
    int ret;

    ESP_ClearRxBuffer();

    Debug_SendString("\r\n>> ");
    Debug_SendString(cmd);

    ESP8266_SendString(cmd);

    ret = ESP_WaitForAny(timeout_ms, succ1, succ2, succ3, NULL, NULL);
    ESP_PrintRxBuffer();

    if (ret == 1)
    {
        Debug_SendString("[OK]\r\n");
        return 1;
    }
    else if (ret == 0)
    {
        Debug_SendString("[FAIL]\r\n");
        return 0;
    }
    else
    {
        Debug_SendString("[TIMEOUT]\r\n");
        return 0;
    }
}

static uint8_t ESP_InitWiFi(void)
{
    char cmd[128];

    Debug_SendString("\r\n=== WiFi Init Start ===\r\n");

    if (!ESP_SendCmdWait("AT\r\n", 1500, "OK", NULL, NULL))
    {
        pass_streak = 0;
        strcpy(last_result, "AT_FAIL");
        OLED_UpdateStatus();
        return 0;
    }

    ESP_SendCmdWait("ATE0\r\n", 1500, "OK", NULL, NULL);

    if (!ESP_SendCmdWait("AT+CWMODE=1\r\n", 2000, "OK", "no change", NULL))
    {
        pass_streak = 0;
        strcpy(last_result, "MODEFAIL");
        OLED_UpdateStatus();
        return 0;
    }

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASSWORD);
    if (!ESP_SendCmdWait(cmd, 20000, "WIFI GOT IP", "OK", NULL))
    {
        pass_streak = 0;
        strcpy(last_result, "WIFIFAIL");
        wifi_ready = 0;
        server_connected = 0;
        OLED_UpdateStatus();
        return 0;
    }

    wifi_ready = 1;
    server_connected = 0;
    pass_streak = 0;
    strcpy(last_result, "WIFI_OK");
    OLED_UpdateStatus();
    Debug_SendString("=== WiFi Connected ===\r\n");
    return 1;
}

static uint8_t ESP_ConnectServer(void)
{
    char cmd[128];

    if (!wifi_ready)
    {
        pass_streak = 0;
        Debug_SendString("WiFi not ready, send 'w' first.\r\n");
        strcpy(last_result, "NO_WIFI");
        OLED_UpdateStatus();
        return 0;
    }

    Debug_SendString("\r\n=== TCP Connect Start ===\r\n");

    ESP_SendCmdWait("AT+CIPCLOSE\r\n", 2000, "OK", "ERROR", "link is not valid");

    if (!ESP_SendCmdWait("AT+CIPMUX=0\r\n", 2000, "OK", NULL, NULL))
    {
        pass_streak = 0;
        strcpy(last_result, "MUX_FAIL");
        OLED_UpdateStatus();
        return 0;
    }

    if (!ESP_SendCmdWait("AT+CIPMODE=0\r\n", 2000, "OK", NULL, NULL))
    {
        pass_streak = 0;
        strcpy(last_result, "MODE0FAIL");
        OLED_UpdateStatus();
        return 0;
    }

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", SERVER_IP, SERVER_PORT);
    if (!ESP_SendCmdWait(cmd, 10000, "CONNECT", "ALREADY CONNECTED", "OK"))
    {
        pass_streak = 0;
        strcpy(last_result, "TCP_FAIL");
        server_connected = 0;
        OLED_UpdateStatus();
        return 0;
    }

    server_connected = 1;
    pass_streak = 0;
    strcpy(last_result, "TCP_OK");
    OLED_UpdateStatus();
    Debug_SendString("=== Server Connected ===\r\n");
    return 1;
}

/* =============================================================================
 *  门锁执行机构：舵机（PWM 角度控制）
 * =============================================================================
 *  ★ 关于"舵机 vs RGB 灯"的取舍（这个反复值得记录）★
 *    一度想用 WS2813E RGB 灯代替舵机，理由是舵机启动电流大、
 *    必须配独立 5V 电源。但实际拿到 RGB 模块后发现：
 *      · 模块用的是 **JST-XH 4P 插座**，插针非常松，一碰就掉 → 演示不可靠
 *      · 而舵机的 **3P 杜邦母头** 插上去很紧，反而更稳
 *    ⇒ **"接插件可靠性"是选型时容易被忽略、但实际很关键的因素**。
 *      最终决定：仍用舵机。
 *
 *  ⚠ 舵机必须在 PB0（TIM3_CH3，无重映射）⇒ 那个位置的跳帽必须拔掉。
 *
 *  ⚠ 供电注意：SG90 约 100~250mA，MG996R 堵转可达 2.5A。
 *    从开发板 5V 取电时，舵机启动可能拉低电压导致单片机复位。
 *    **强烈建议舵机用独立 5V 电源，并与板子共地。**
 * ========================================================================== */

/* =============================================================================
 *  舵机相关代码已移除
 * =============================================================================
 *  ★ 为什么删掉而不是保留 ★
 *    舵机最终没能驱动成功（寄存器诊断证明 PWM 配置完全正确，问题在片外：
 *    信号线接触或舵机本身），门锁指示改成 OLED 显示。保留不用的舵机代码：
 *      ① 白白占 Flash（MDK-Lite 对代码体积敏感，之前已经卡过 32KB 限制）
 *      ② 留下死代码，以后看代码的人会以为它还在用
 *    ⇒ 直接删干净，让代码和实际功能保持一致。
 *
 *  ⚠ PB0 的处理：
 *    原来它配成 TIM3_CH3 复用推挽驱动舵机；现在不再启动 TIM3 的 PWM 输出。
 *    舵机若还插着会因为收到无效信号而轻微嗡响 —— **直接拔掉舵机即可**。
 * ========================================================================== */



/* =============================================================================
 *  门锁指示：OLED 屏幕（"开锁"是逻辑上的，不再驱动物理执行机构）
 * =============================================================================
 *  ★ 为什么最终放弃舵机、改用 OLED ★
 *    舵机实际调试中遇到一连串问题：
 *      · 信号线颜色无标识（红/橙/黑），只能靠通用惯例猜
 *      · PB0 上有跳帽，拔掉才能用（而且拔了之后仍不转）
 *      · 上电后有嗡嗡声但不动作 —— 典型"通电但控制信号没生效"
 *      · 寄存器诊断确认 PWM 配置完全正确（CEN=1、50Hz、PB0=AF_PP），
 *        说明问题在**片外**（舵机本身或信号线接触）
 *
 *    而 OLED **一直在稳定工作**（项目全程都在用它显示状态），
 *    只要把"判定结果"显示清楚，演示效果同样直观，
 *    而且**零额外硬件、零调试风险**。
 *  ⇒ 工程判断：**在时间有限时，把资源投入到"确定能做出来"的方案上。**
 *    功能上"允许通行 / 拒绝"的语义完全等价，只是反馈形式从"动作"变成"显示"。
 * ========================================================================== */
static void Door_Open(void)
{
    door_is_open = 1;
    strcpy(last_result, "PASS");       /* 让 OLED 显示 *** PASS *** */
    OLED_UpdateStatus();
    Debug_SendString("Door Open\r\n");
}

static void Door_Close(void)
{
    door_is_open = 0;
    strcpy(last_result, "CLOSE");      /* 回到待机显示 */
    last_sim_permille = 0U;            /* 清掉上一次的相似度 */
    OLED_UpdateStatus();
    Debug_SendString("Door Close\r\n");
}

static uint16_t Mic_ReadOnce(void)
{
    uint16_t value = 0;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
    {
        value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return value;
}

/* =============================================================================
 *  【升级】语音采集三件套 —— 从 250Hz 软件采样 改为 8kHz 硬件采集
 * =============================================================================
 *  为什么要改（这是整个项目升级的核心）：
 *    原版这三个函数都调用 Mic_ReadOnce()（软件触发单次转换 + HAL_Delay(2)），
 *    等效采样率 250Hz，奈奎斯特极限只有 125Hz，而人声基频是 85~255Hz
 *    ⇒ **采到的数据里连人声基频都没有**，只剩"音量包络"。
 *    这正是原论文要写"云端声纹仅作接口验证"的根本原因：数据不支持声纹。
 *
 *  现在它们改为调用 audio_capture.c 的 8kHz 硬件采集：
 *    · 采样由 TIM2 硬件触发（精度由晶振保证，实测 7996~8000Hz）
 *    · 数据由 DMA 自动搬运到内存（CPU 完全不参与）
 *    · 触发检测、基线估计都在滚动缓冲上做
 *  ⇒ 采到的是**真正的 1 秒语音波形（8000 点）**，可以做 MFCC。
 * ========================================================================== */

static uint16_t MeasureBaseline(void)
{
    /* 在 8kHz 滚动缓冲上取 200ms 的均值作为静音基线。
     * 原来是采 24 次 × 每 4ms（约 96ms）；现在按"时间"取 200ms，更稳。 */
    return audio_measure_baseline();
}

static uint8_t WaitForVoiceTrigger(void)
{
    strcpy(last_result, "WAIT");
    OLED_UpdateStatus();

    /* 用"短时能量 RMS"的 VAD（见 audio_capture.h 的说明）：
     *   分成 10ms 一帧算能量，连续 3 帧超过"噪声底×3"才算有效语音。
     *   ⚠ 不要改回"连续 N 个样本偏离基线" —— 8kHz 下波形振荡会让它永不触发。 */
    if (audio_wait_trigger())
    {
        return 1;
    }
    return 0;
}

static void CaptureVoiceWindow(void)
{
    strcpy(last_result, "CAPT");
    OLED_UpdateStatus();

    /* 采 1 秒 = 8000 点真实音频（原来是 220 点 × 4ms，只有音量包络）。
     *
     * ★ 这里**故意不复制数据** ★
     *   audio_capture_window() 返回的是它内部静态缓冲区的指针（16KB）。
     *   如果为了"好看"再复制一份到 adc_samples[8000]，就要**再花 16KB RAM**，
     *   而 RET6 只有 64KB，加上 DMA 的 32KB 滚动缓冲就快满了。
     *   ⇒ 后续 ExtractVoiceFeature() 直接从那个指针读（只读，不持有）。
     *
     * 代价：窗口数据只在"下一次采集"之前有效。对这个流程（采完立刻提特征）
     *       完全够用 —— 这是典型的"用生命周期换内存"的取舍。 */
    (void)audio_capture_window();
}

static VoiceFeature_t ExtractVoiceFeature(void)
{
    VoiceFeature_t f;
    uint32_t sum = 0;
    uint32_t energy_sum = 0;
    uint32_t i;
    uint32_t n;
    uint32_t seg_size;
    const uint16_t *buf;

    /* 直接从采集模块的缓冲区读（不再经过 adc_samples，省 16KB RAM） */
    buf = audio_capture_window();
    n   = audio_window_len();
    if (buf == 0 || n == 0U)
    {
        /* 采集失败：返回全 0 的特征，调用方按"无有效语音"处理 */
        f.mean = 0U; f.min = 0U; f.max = 0U; f.p2p = 0U; f.energy = 0U;
        f.seg1 = 0U; f.seg2 = 0U; f.seg3 = 0U; f.seg4 = 0U;
        f.seg5 = 0U; f.seg6 = 0U; f.seg7 = 0U; f.seg8 = 0U;
        return f;
    }

    f.min = 4095U;
    f.max = 0U;

    f.seg1 = 0U; f.seg2 = 0U; f.seg3 = 0U; f.seg4 = 0U;
    f.seg5 = 0U; f.seg6 = 0U; f.seg7 = 0U; f.seg8 = 0U;

    for (i = 0; i < n; i++)
    {
        sum += buf[i];

        if (buf[i] < f.min) f.min = buf[i];
        if (buf[i] > f.max) f.max = buf[i];
    }

    f.mean = (uint16_t)(sum / n);
    f.p2p = f.max - f.min;

    seg_size = n / 8U;

    for (i = 0; i < n; i++)
    {
        uint32_t diff;

        if (buf[i] >= f.mean)
            diff = buf[i] - f.mean;
        else
            diff = f.mean - buf[i];

        energy_sum += diff;

        if (i < seg_size) f.seg1 += diff;
        else if (i < seg_size * 2U) f.seg2 += diff;
        else if (i < seg_size * 3U) f.seg3 += diff;
        else if (i < seg_size * 4U) f.seg4 += diff;
        else if (i < seg_size * 5U) f.seg5 += diff;
        else if (i < seg_size * 6U) f.seg6 += diff;
        else if (i < seg_size * 7U) f.seg7 += diff;
        else f.seg8 += diff;
    }

    f.energy = energy_sum;
    return f;
}

/* =============================================================================
 *  SendAudioToESP32 — 把采集到的 1 秒音频打包发往 ESP32-S3（双片通信）
 * =============================================================================
 *  为什么这样设计（音质 vs 带宽的权衡）
 *  ---------------------------------------------------------------------------
 *  原始数据：8000 点 × 16bit = 16000 字节 → 115200 下要 1.4 秒，太慢。
 *
 *  ★ 只做"位深压缩"，**不做抽取（降采样）** ★
 *    最初我设计的是"2 倍抽取到 4kHz + 8bit 量化"，能压到 4000 字节（0.35 秒）。
 *    但后来发现这**会毁掉声纹识别**：
 *      4kHz 采样 ⇒ 奈奎斯特上限只有 2kHz，
 *      而**区分不同说话人的关键信息（第二、第三共振峰）大量分布在 2~4kHz**。
 *      丢掉这段等于丢掉了"音色"里最有辨识度的部分。
 *    ⇒ 保留 8kHz（奈奎斯特 4kHz，覆盖完整语音频段），
 *      只做 8bit 量化：8000 字节 / 约 0.7 秒。
 *      **多花 0.35 秒，换来"能做真正的声纹"** —— 这个取舍是值得的。
 *
 *  8bit 量化（取 12 位 ADC 的高 8 位）
 *  ---------------------------------------------------------------------------
 *    12bit → 8bit，动态范围 72dB → 48dB。
 *    MFCC 取的是"对数能量"，对量化噪声本来就不敏感；
 *    而且我们关心的是**频谱形状**（音色），不是绝对幅度精度。
 *    解码方把 8bit 左移 4 位即可还原到 12bit 量程。
 *
 *  帧格式（见 uart_proto.h）：
 *     先发一帧 META（告诉 ESP32：采样率/点数/块数），再分块发 DATA。
 *     为什么先发 META：ESP32 可以据此**预分配缓冲区**并根据"总块数"
 *     判断有没有丢块，比"边收边猜"可靠。
 * ========================================================================== */
#define AUDIO_SEND_RATE      AUDIO_SAMPLE_RATE_HZ          /* 8000Hz，不抽取 */
#define AUDIO_SEND_COUNT     AUDIO_CAPTURE_LEN             /* 8000 点 */
#define AUDIO_CHUNK_SAMPLES  256U          /* 每帧 256 个 8bit 样本 = 256 字节 */
#define AUDIO_CHUNK_COUNT    ((AUDIO_SEND_COUNT + AUDIO_CHUNK_SAMPLES - 1U) / AUDIO_CHUNK_SAMPLES)

static uint8_t s_audio_pkt[UART_MAX_FRAME];    /* 组帧用的临时缓冲 */
static uint8_t s_audio_8bit[AUDIO_CHUNK_SAMPLES];   /* 8bit 量化后的块 */

static void SendAudioToESP32(void)
{
    const uint16_t *raw = audio_capture_window();
    const uint32_t  n   = audio_window_len();

    if (raw == 0 || n == 0U)
    {
        Debug_SendString("[TX] no audio window\r\n");
        return;
    }

    /* ---- ① 先发 META 帧：告诉 ESP32 接下来会收到什么 ------------------- */
    uint8_t meta[UART_META_LEN];
    meta[0] = (uint8_t)(AUDIO_SEND_RATE & 0xFFU);
    meta[1] = (uint8_t)((AUDIO_SEND_RATE >> 8) & 0xFFU);
    meta[2] = (uint8_t)((AUDIO_SEND_RATE >> 16) & 0xFFU);
    meta[3] = (uint8_t)((AUDIO_SEND_RATE >> 24) & 0xFFU);
    meta[4] = (uint8_t)(AUDIO_SEND_COUNT & 0xFFU);
    meta[5] = (uint8_t)((AUDIO_SEND_COUNT >> 8) & 0xFFU);
    meta[6] = (uint8_t)((AUDIO_SEND_COUNT >> 16) & 0xFFU);
    meta[7] = (uint8_t)((AUDIO_SEND_COUNT >> 24) & 0xFFU);
    meta[8]  = (uint8_t)(AUDIO_CHUNK_COUNT & 0xFFU);
    meta[9]  = (uint8_t)((AUDIO_CHUNK_COUNT >> 8) & 0xFFU);
    meta[10] = 16U;   /* 原始位深（ADC 12 位，打包时按 16 位描述）*/
    meta[11] = 8U;    /* 实际传输位深（8bit 量化）*/

    uint32_t len = uart_build_frame(s_audio_pkt, UART_CMD_AUDIO_META,
                                    uart_next_seq(), meta, UART_META_LEN);
    if (len > 0U)
    {
        HAL_UART_Transmit(&huart3, s_audio_pkt, (uint16_t)len, 1000);
    }

    Debug_Printf("[TX] META rate=%lu pts=%lu chunks=%lu bit=%u\r\n",
                 (unsigned long)AUDIO_SEND_RATE,
                 (unsigned long)AUDIO_SEND_COUNT,
                 (unsigned long)AUDIO_CHUNK_COUNT, 8U);

    /* ---- ② 分块发音频 ------------------------------------------------ */
    uint32_t sent = 0;
    uint32_t chunks = 0;

    while (sent < AUDIO_SEND_COUNT)
    {
        const uint32_t remain = AUDIO_SEND_COUNT - sent;
        const uint32_t this_chunk = (remain > AUDIO_CHUNK_SAMPLES)
                                    ? AUDIO_CHUNK_SAMPLES : remain;

        /* 8bit 量化：取原始样本的高 8 位（右移 4）。
         * 12 位 ADC（0~4095）→ 8 位（0~255），4096/256 = 16 = 2^4，
         * 解码方左移 4 位即可还原量程。 */
        for (uint32_t i = 0; i < this_chunk; i++)
        {
            const uint32_t src = sent + i;
            const uint16_t v = (src < n) ? raw[src] : 0U;
            s_audio_8bit[i] = (uint8_t)(v >> 4);
        }

        len = uart_build_frame(s_audio_pkt, UART_CMD_AUDIO_DATA,
                               uart_next_seq(), s_audio_8bit,
                               (uint16_t)this_chunk);
        if (len == 0U)
        {
            Debug_SendString("[TX] frame build FAILED\r\n");
            return;
        }

        if (HAL_UART_Transmit(&huart3, s_audio_pkt, (uint16_t)len, 1000) != HAL_OK)
        {
            Debug_SendString("[TX] uart FAILED\r\n");
            return;
        }

        sent   += this_chunk;
        chunks++;
    }

    Debug_Printf("[TX] audio sent: %lu chunks, %lu bytes\r\n",
                 (unsigned long)chunks,
                 (unsigned long)(AUDIO_SEND_COUNT + chunks * UART_OVERHEAD));
}

static uint8_t ESP_CloudVerifyOnce(void)
{
    int ret;
    char send_cmd[32];
    char payload[192];
    VoiceFeature_t feat;
    uint16_t baseline;

    if (!server_connected)
    {
        pass_streak = 0;
        Debug_SendString("Server not connected, send 's' first.\r\n");
        strcpy(last_result, "NO_SRV");
        OLED_UpdateStatus();
        return 0;
    }

    Debug_SendString("\r\n=== Trigger Feature Verify Start ===\r\n");

    baseline = MeasureBaseline();
    Debug_Printf("baseline=%u\r\n", baseline);

    if (!WaitForVoiceTrigger())
    {
        pass_streak = 0;
        Debug_SendString("[TRIGGER TIMEOUT]\r\n");
        strcpy(last_result, "NO_VOICE");
        OLED_UpdateStatus();
        return 0;
    }

    Debug_SendString("[VOICE TRIGGERED]\r\n");

    CaptureVoiceWindow();
    feat = ExtractVoiceFeature();

    Debug_Printf("mean=%u min=%u max=%u p2p=%u energy=%lu\r\n",
                 feat.mean, feat.min, feat.max, feat.p2p, (unsigned long)feat.energy);
    Debug_Printf("seg=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                 (unsigned long)feat.seg1, (unsigned long)feat.seg2,
                 (unsigned long)feat.seg3, (unsigned long)feat.seg4,
                 (unsigned long)feat.seg5, (unsigned long)feat.seg6,
                 (unsigned long)feat.seg7, (unsigned long)feat.seg8);

    snprintf(payload, sizeof(payload),
             "FEAT,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
             feat.mean,
             feat.min,
             feat.max,
             feat.p2p,
             (unsigned long)feat.energy,
             (unsigned long)feat.seg1,
             (unsigned long)feat.seg2,
             (unsigned long)feat.seg3,
             (unsigned long)feat.seg4,
             (unsigned long)feat.seg5,
             (unsigned long)feat.seg6,
             (unsigned long)feat.seg7,
             (unsigned long)feat.seg8);

    snprintf(send_cmd, sizeof(send_cmd),
             "AT+CIPSEND=%u\r\n",
             (unsigned int)strlen(payload));

    ESP_ClearRxBuffer();
    Debug_Printf("\r\n>> %s", send_cmd);
    ESP8266_SendString(send_cmd);

    ret = ESP_WaitForAny(4000, ">", NULL, NULL, NULL, NULL);
    ESP_PrintRxBuffer();

    if (ret != 1)
    {
        pass_streak = 0;
        Debug_SendString("[CIPSEND FAIL]\r\n");
        strcpy(last_result, "SENDFAIL");
        OLED_UpdateStatus();
        return 0;
    }

    ESP_ClearRxBuffer();
    Debug_Printf("\r\n>> %s", payload);
    ESP8266_SendString(payload);

    ret = ESP_WaitForAny(6000, "PASS", "FAIL", NULL, NULL, NULL);
    ESP_PrintRxBuffer();

    if (ret == 1)
    {
        if (strstr(esp_rx_buf, "PASS"))
        {
            if (pass_streak == 0)
            {
                pass_streak = 1;
                strcpy(last_result, "PASS1");
                OLED_UpdateStatus();
                Debug_SendString("[PASS 1/2]\r\n");
                return 1;
            }
            else
            {
                pass_streak = 0;
                strcpy(last_result, "PASS2");
                OLED_UpdateStatus();
                Door_Open();
                Debug_SendString("[PASS 2/2 OPEN]\r\n");
                return 1;
            }
        }
        else if (strstr(esp_rx_buf, "FAIL"))
        {
            pass_streak = 0;
            strcpy(last_result, "FAIL");
            OLED_UpdateStatus();
            Door_Close();
            Debug_SendString("[FEATURE FAIL]\r\n");
            return 1;
        }
    }

    pass_streak = 0;
    strcpy(last_result, "TIMEOUT");
    OLED_UpdateStatus();
    Debug_SendString("[FEATURE TIMEOUT/ERROR]\r\n");
    return 0;
}

/* =============================================================================
 *  I2C_Diag — OLED / I2C 总线诊断（'D' 命令）
 * =============================================================================
 *  为什么需要这个：
 *    "OLED 全黑"可能是好几层原因，光看现象分不清：
 *      ① 没供电（VCC/GND 没接）
 *      ② I2C 总线上没有器件（SDA/SCL 接反、线松）
 *      ③ 器件在，但地址不对（模块可能是 0x3C 或 0x3D）
 *      ④ 器件在、地址对，但初始化序列没发出去
 *    本函数**主动扫描整条 I2C 总线**，把①②③区分开。
 *
 *  ⚠ 特别提醒 STM32F1 的 I2C 有个著名坑：
 *    如果主机在从机没应答时被复位/中断，I2C 外设会**锁死**（BUSY 一直为 1），
 *    之后所有通信都失败，而且**不报错**（旧代码里 OLED_Init 就没检查返回值）。
 *    扫描前先做一次"总线恢复"（发 9 个时钟脉冲把从机状态机踢回空闲）。
 * ========================================================================== */
static void I2C_Diag(void)
{
    HAL_StatusTypeDef rc;

    Debug_SendString("\r\n=== I2C / OLED DIAG ===\r\n");

    /* ---- ① 引脚配置检查 ----
     * PB6/PB7 在 CRL 里：PB6 是 bit24~27、PB7 是 bit28~31。
     * 复用开漏（I2C 要求）应为 **0xE**（CNF=11 开漏复用 + MODE=10 快速）。 */
    Debug_Printf("[I2C] GPIOB CRL = 0x%08lX\r\n", (unsigned long)GPIOB->CRL);
    Debug_Printf("[I2C]   PB6(SCL) cfg = 0x%X (expect 0xE for AF open-drain)\r\n",
                 (unsigned)((GPIOB->CRL >> 24) & 0xFU));
    Debug_Printf("[I2C]   PB7(SDA) cfg = 0x%X (expect 0xE for AF open-drain)\r\n",
                 (unsigned)((GPIOB->CRL >> 28) & 0xFU));

    Debug_Printf("[I2C] I2C1 CR1=0x%04X CR2=0x%04X SR2=0x%04X\r\n",
                 (unsigned)I2C1->CR1, (unsigned)I2C1->CR2,
                 (unsigned)I2C1->SR2);
    /* SR2 的 bit1 = BUSY；BUSY=1 说明总线被占死 */
    if ((I2C1->SR2 & 0x02U) != 0U) {
        Debug_SendString("[I2C] !! BUSY=1 -> bus is stuck (peripheral locked)\r\n");
    } else {
        Debug_SendString("[I2C] BUSY=0 -> bus idle\r\n");
    }
    Debug_Printf("[I2C] APB1ENR=0x%08lX (I2C1EN=%u)\r\n",
                 (unsigned long)RCC->APB1ENR,
                 (unsigned)((RCC->APB1ENR >> 21) & 1U));

    /* ---- ② 总线恢复：手动发 9 个 SCL 脉冲 ----
     * 做法：把 PB6/PB7 临时配成普通开漏 GPIO，
     * 拉低 SDA → 打 9 个 SCL → 发 STOP → 再恢复成 I2C 复用。 */
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
        gi.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏输出 */
        gi.Pull  = GPIO_NOPULL;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gi);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  /* SDA 低 */
        for (uint32_t i = 0; i < 9U; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  /* SCL 低 */
            for (volatile uint32_t d = 0; d < 200U; d++) { }
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /* SCL 高 */
            for (volatile uint32_t d = 0; d < 200U; d++) { }
        }
        /* STOP 条件：SCL 高时 SDA 由低变高 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < 200U; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < 200U; d++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < 400U; d++) { }

        /* 恢复成 I2C 复用开漏 */
        gi.Mode  = GPIO_MODE_AF_OD;
        gi.Pull  = GPIO_PULLUP;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gi);
        Debug_SendString("[I2C] bus recovery pulses sent (9 clocks + STOP)\r\n");
    }

    /* ---- ③ 扫描整条总线 ----
     * 7 位地址范围 0x08 ~ 0x77。用"发 1 字节"探测是否有 ACK。 */
    Debug_SendString("[I2C] scanning addresses 0x08 ~ 0x77 ...\r\n");
    {
        uint8_t found = 0;

        for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
            const uint16_t dev = (uint16_t)(addr << 1);   /* HAL 要 8 位地址 */
            uint8_t dummy = 0x00U;

            rc = HAL_I2C_Master_Transmit(&hi2c1, dev, &dummy, 1U, 20U);
            if (rc == HAL_OK) {
                Debug_Printf("[I2C]   FOUND device at 7-bit addr 0x%02X "
                             "(HAL 8-bit 0x%02X)\r\n",
                             (unsigned)addr, (unsigned)dev);
                found++;
            }
            /* 探测失败是正常的（大多数地址没器件），不打印，避免刷屏 */
        }

        if (found == 0U) {
            Debug_SendString("[I2C]   NO device found on the bus !!\r\n");
            Debug_SendString("[I2C]   => check wiring: OLED VCC/GND, SCL->PB6, SDA->PB7\r\n");
        } else {
            Debug_Printf("[I2C] scan done: %u device(s) found\r\n", (unsigned)found);
            Debug_SendString("[I2C]   SSD1306 is usually 0x3C or 0x3D. "
                             "Code uses 0x78 (8-bit) == 0x3C (7-bit)\r\n");
        }
    }

    /* ---- ④ 往 OLED 地址发一次真实数据，看返回码 ---- */
    {
        const uint8_t probe[2] = { 0x00U, 0xAEU };   /* 关显示命令 */
        rc = HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR, (uint8_t *)probe,
                                     2U, 100U);
        Debug_Printf("[I2C] write test to 0x%02X (OLED) -> %s\r\n",
                     (unsigned)OLED_I2C_ADDR,
                     (rc == HAL_OK) ? "HAL_OK (OLED responds!)" :
                     (rc == HAL_ERROR) ? "HAL_ERROR (NACK / no device)" :
                     (rc == HAL_BUSY) ? "HAL_BUSY (bus stuck)" :
                     (rc == HAL_TIMEOUT) ? "HAL_TIMEOUT (SCL/SDA held low?)"
                                         : "unknown");
        if (rc == HAL_OK) {
            Debug_SendString("[I2C] => I2C path is FINE. "
                             "If screen is still black, re-run OLED_Init.\r\n");
        }
    }

    /* ★ 显示 I2C 通信累计统计（这是判断"屏幕为什么黑"的关键数据）★ */
    Debug_Printf("[I2C] total ok=%lu  err=%lu\r\n",
                 (unsigned long)OLED_I2C_OkCount(),
                 (unsigned long)OLED_I2C_ErrCount());
    if (OLED_I2C_ErrCount() > 0UL) {
        Debug_SendString("[I2C]   !! there WERE errors -> likely power dip "
                         "or loose wire during operation\r\n");
    } else {
        Debug_SendString("[I2C]   no errors so far -> bus is clean\r\n");
    }

    Debug_SendString("=== I2C DIAG DONE ===\r\n");
}

static void OLED_UpdateStatus(void)
{
    char line2[17];
    char line3[17];
    char line4[17];

    OLED_Clear();

    /* ★ 第 1 行：把"判定结果"放到最显眼的位置 ★
     *   为什么改：门锁指示从舵机改成 OLED 之后，
     *   屏幕就成了**唯一的反馈渠道** —— 所以结果要放大到标题行，
     *   而不是塞在第 4 行的小字里。面试演示时一眼能看清。 */
    if (strcmp(last_result, "PASS") == 0) {
        OLED_ShowString(1, 1, "*** PASS ***");        /* 允许通行 */
    } else if (strcmp(last_result, "FAIL") == 0) {
        OLED_ShowString(1, 1, "--- FAIL ---");        /* 拒绝 */
    } else if (strcmp(last_result, "VERIFY") == 0) {
        OLED_ShowString(1, 1, "  VERIFYING ");
    } else if (strcmp(last_result, "WAIT") == 0) {
        OLED_ShowString(1, 1, "  LISTENING ");
    } else if (strcmp(last_result, "NO_VOICE") == 0) {
        OLED_ShowString(1, 1, "  NO VOICE  ");
    } else {
        OLED_ShowString(1, 1, " VOICE LOCK V2 ");
    }

    /* 第 2 行：最近一次判定的**相似度**（最能体现算法在工作）
     * 第 3 行：判定结论的文字说明
     *   ⚠ 阈值（threshold）存在 ESP32 侧、没有通过协议回传，
     *     所以这里不显示阈值，改用一句结论说明 —— 演示效果一样清楚。 */
    if (last_sim_permille > 0U) {
        snprintf(line2, sizeof(line2), "SIM:%u.%03u",
                 (unsigned)(last_sim_permille / 1000U),
                 (unsigned)(last_sim_permille % 1000U));

        if (strcmp(last_result, "PASS") == 0) {
            strcpy(line3, "ACCESS GRANTED");       /* 16 字符正好占满一行 */
        } else if (strcmp(last_result, "FAIL") == 0) {
            strcpy(line3, "ACCESS DENIED");
        } else {
            snprintf(line3, sizeof(line3), "R:%s", last_result);
        }
        OLED_ShowString(2, 1, line2);
        OLED_ShowString(3, 1, line3);
    } else {
        /* 还没有判定结果时，显示运行状态 */
        snprintf(line2, sizeof(line2), "D:%s W:%s",
                 door_is_open ? "OPEN " : "CLOSE",
                 wifi_ready ? "OK" : "NO");
        snprintf(line3, sizeof(line3), "A:%4u", mic_adc_value);
        OLED_ShowString(2, 1, line2);
        OLED_ShowString(3, 1, line3);
    }

    /* 第 4 行：状态摘要 */
    snprintf(line4, sizeof(line4), "R:%-10s", last_result);
    OLED_ShowString(4, 1, line4);
}

static void ShowHelp(void)
{
    Debug_SendString("\r\n========================\r\n");
    Debug_SendString("System Ready V2\r\n");
    Debug_SendString("A -> ESP8266 AT\r\n");
    Debug_SendString("R -> ESP8266 Reset\r\n");
    Debug_SendString("O -> Door Open\r\n");
    Debug_SendString("C -> Door Close\r\n");
    Debug_SendString("W -> Connect WiFi\r\n");
    Debug_SendString("S -> Connect Server\r\n");
    Debug_SendString("V -> Trigger + Send 8Seg Feature\r\n");
    Debug_SendString("T -> [NEW] Audio capture self-test (8kHz)\r\n");
    Debug_SendString("U -> [NEW] FULL VERIFY: capture+upload+verdict\r\n");
    Debug_SendString("P -> upload only (no wait for verdict)\r\n");
    Debug_SendString("K -> [NEW] Acoustic calibration (quiet vs speech)\r\n");
    Debug_SendString("X -> [NEW] USART3 hardware diag (TX path check)\r\n");
    Debug_SendString("L -> [NEW] USART3 loopback test (jumper PB10-PB11)\r\n");
    Debug_SendString("D -> [NEW] I2C/OLED diag (scans the bus)\r\n");
    Debug_SendString("Two PASS needed to OPEN\r\n");
    Debug_SendString("H -> Help\r\n");
    Debug_SendString("========================\r\n");
}

/* =============================================================================
 *  audio_selftest — 8kHz 采集自检（新增，用于验证采样链路）
 * =============================================================================
 *  为什么要这个自检：
 *    采样是整个项目的**源头**。如果采样率不对、DMA 没跑、或者信号被削顶，
 *    后面所有特征和识别都是错的 —— 而且那种错误很隐蔽（看起来"有数据"）。
 *    所以先做一个能一眼看出对错的检查：
 *      · DMA 位置在不在动      → 说明定时器+ADC+DMA 都在工作
 *      · min/max 有没有变化    → 说明信号不是死值（恒定 = 没接麦克风或没触发）
 *      · 均值接近 2048         → 说明 MAX9814 的直流偏置正确（VDD/2 = 1.65V）
 *      · 峰峰值有多大          → 判断灵敏度是否够（对着说话应该明显变化）
 *  ⚠ 本函数用 HAL_Delay 只是为了"隔 1ms 取一个点"方便观察，
 *    真正的高精度采集在 audio_capture_window() 里（只等 DMA 计数，不看 tick）。
 * ========================================================================== */
static void Audio_SelfTest(void)
{
    const uint32_t STEP = AUDIO_SAMPLE_RATE_HZ / 1000U;   /* 1ms = 8 个样本 */

    if (!audio_is_running())
    {
        Debug_SendString("[AUDIO] not running, call audio_init() first\r\n");
        return;
    }

    Debug_SendString("\r\n=== AUDIO SELF-TEST (8kHz) ===\r\n");
    Debug_SendString("Speak or clap; watch min/max/p2p change\r\n");

    /* ---- 先做一次"采样率实测"：读 1 秒内 DMA 前进了多少样本 ----
     * 这是最直接的判据：8kHz 下 1 秒应该前进 8000 个样本。
     * 为什么要测：只看 dma_pos 的绝对值看不出速率（会回绕，且中间夹着
     * 串口打印的等待时间），必须用"时间 + 增量"来算。 */
    {
        const uint16_t p0 = audio_dma_pos();
        const uint32_t t0 = HAL_GetTick();
        HAL_Delay(1000);
        const uint16_t p1 = audio_dma_pos();
        const uint32_t dt = HAL_GetTick() - t0;
        const uint32_t inc = (uint32_t)((p1 - p0) & AUDIO_DMA_BUF_MASK);
        Debug_Printf("[AUDIO] === sample rate test ===\r\n");
        Debug_Printf("[AUDIO] dma advanced %lu samples in %lu ms\r\n",
                     (unsigned long)inc, (unsigned long)dt);
        if (dt > 0U) {
            Debug_Printf("[AUDIO] measured rate = %lu Hz (expect %lu)\r\n",
                         (unsigned long)(inc * 1000U / dt),
                         (unsigned long)AUDIO_SAMPLE_RATE_HZ);
        }
        /* 直接读寄存器核实定时器/ADC 状态（不依赖任何库函数） */
        Debug_Printf("[AUDIO] TIM2 CR1=0x%04X PSC=%u ARR=%u CCR2=%u\r\n",
                     (unsigned)TIM2->CR1, (unsigned)TIM2->PSC,
                     (unsigned)TIM2->ARR, (unsigned)TIM2->CCR2);
        Debug_Printf("[AUDIO] ADC1 CR1=0x%04X CR2=0x%04X\r\n",
                     (unsigned)ADC1->CR1, (unsigned)ADC1->CR2);
        Debug_Printf("[AUDIO] DMA1_CCR1=0x%04X CNDTR1=%u\r\n",
                     (unsigned)DMA1_Channel1->CCR, (unsigned)DMA1_Channel1->CNDTR);
        Debug_Printf("[AUDIO] RCC APB1ENR=0x%08lX APB2ENR=0x%08lX AHBENR=0x%08lX\r\n",
                     (unsigned long)RCC->APB1ENR, (unsigned long)RCC->APB2ENR,
                     (unsigned long)RCC->AHBENR);
    }

    for (uint32_t loop = 0; loop < 5U; loop++)   /* 连续测 5 轮，每轮 500ms */
    {
        uint16_t lo = 4095U, hi = 0U;
        uint32_t sum = 0, n = 0;
        uint16_t pos = audio_dma_pos();

        for (uint32_t i = 0; i < 500U; i++)      /* 500 个点 = 500ms */
        {
            const uint16_t s = audio_peek(pos);   /* 直接读 DMA 滚动缓冲 */
            if (s < lo) lo = s;
            if (s > hi) hi = s;
            sum += s; n++;
            pos = (uint16_t)(pos + STEP);           /* 前进 1ms */
            HAL_Delay(1);
        }

        const uint16_t avg = (uint16_t)(sum / n);
        Debug_Printf("[AUDIO] round %lu: avg=%u min=%u max=%u p2p=%u dma_pos=%u\r\n",
                     (unsigned long)(loop + 1U), avg, lo, hi, (uint16_t)(hi - lo),
                     audio_dma_pos());

        /* 判断提示：直流偏置应该在 2048 附近（MAX9814 @3.3V -> 中心 1.65V）*/
        if ((avg < 1500U) || (avg > 2600U))
        {
            Debug_SendString("  ! BIAS ABNORMAL: check MAX9814 powered by 3.3V "
                             "and OUTPUT wired to PA1\r\n");
        }
        if ((uint16_t)(hi - lo) < 20U)
        {
            Debug_SendString("  ! FLAT SIGNAL: quiet is normal; if always flat, "
                             "check mic wiring\r\n");
        }
    }
    Debug_SendString("=== SELF-TEST DONE ===\r\n");
}

/* =============================================================================
 *  Audio_Calibrate — 声学标定（'k' 命令）
 * =============================================================================
 *  为什么需要这个：
 *    触发阈值是"噪声底 × 倍数"自适应出来的，但**如果标定时环境不干净
 *    （或者用户已经在说话），阈值会被污染得很高** → 永远不触发。
 *    实测就遇到过：两次测噪声底差了 2.6 倍（104 vs 273）。
 *    所以需要一个工具，把"安静时多响"和"说话时多响"分别量出来，
 *    才能判断是阈值定得不合理，还是麦克风/环境本身有问题。
 *
 *  输出：静音峰值 RMS、说话峰值 RMS，以及建议的触发阈值。
 * ========================================================================== */
static void Audio_Calibrate(void)
{
    if (!audio_is_running())
    {
        Debug_SendString("[CAL] audio not running\r\n");
        return;
    }

    Debug_SendString("\r\n=== ACOUSTIC CALIBRATION ===\r\n");
    Debug_SendString("[CAL] STEP 1/2: keep QUIET for ~1.5s ...\r\n");
    HAL_Delay(300);

    const uint16_t quiet = audio_measure_peak_rms(1500);
    Debug_Printf("[CAL] quiet  peak rms = %u\r\n", (unsigned)quiet);

    Debug_SendString("[CAL] STEP 2/2: SPEAK LOUDLY for ~3s NOW!\r\n");
    const uint16_t talk = audio_measure_peak_rms(3000);
    Debug_Printf("[CAL] speech peak rms = %u\r\n", (unsigned)talk);

    const uint32_t suggest = ((uint32_t)quiet + (uint32_t)talk) / 2U;
    Debug_Printf("[CAL] ---> suggested thresh ~ %lu\r\n",
                 (unsigned long)suggest);

    if (talk < 100U)
    {
        Debug_SendString("[CAL] ! speech too weak: move closer, "
                         "or check GAIN pin (tie to GND = 60dB)\r\n");
    }
    else if (talk < (uint32_t)quiet * 2U)
    {
        Debug_SendString("[CAL] ! speech barely above noise: "
                         "too noisy here, or mic too far\r\n");
    }
    else
    {
        const uint32_t ratio10 = ((uint32_t)talk * 10U)
                                 / ((quiet == 0U) ? 1U : (uint32_t)quiet);
        Debug_Printf("[CAL] OK: speech is %lu.%lux the noise floor\r\n",
                     (unsigned long)(ratio10 / 10U),
                     (unsigned long)(ratio10 % 10U));
    }
    Debug_SendString("=== CALIBRATION DONE ===\r\n");
}

/* =============================================================================
 *  Audio_CaptureAndUpload — 完整的"采集 → 上传"链路测试（'u' 命令）
 * =============================================================================
 *  这是**双片通信的端到端验证**：F103 等你说一句话，采 1 秒，
 *  压缩后通过 UART3 发给 ESP32-S3，并在本地也打印特征。
 *
 *  为什么单独做一个命令（而不直接塞进 'v'）：
 *    排查问题时需要**分层验证** —— 先确认"采集+上传"这一段通不通，
 *    再确认 ESP32 侧的"MFCC+推理"对不对。如果混在一起，
 *    出问题时分不清是哪一段的锅。
 * ========================================================================== */
static void Audio_CaptureAndUpload(void)
{
    uint16_t baseline;
    VoiceFeature_t feat;

    if (!audio_is_running())
    {
        Debug_SendString("[CAP] audio not running (did audio_init fail?)\r\n");
        return;
    }

    Debug_SendString("\r\n=== CAPTURE + UPLOAD (8kHz) ===\r\n");

    /* ① 估计静音基线（200ms）＋ 噪声底 RMS
     *    为什么要一起打出来：触发阈值是"噪声底×3"，如果噪声底本身很大
     *    （比如空调/风扇很响），阈值就会被抬得很高导致不触发；
     *    打出来才能一眼看出"是阈值问题还是没说话"。 */
    baseline = MeasureBaseline();
    {
        const uint16_t nrms = audio_measure_noise_rms();
        uint32_t thr = (uint32_t)nrms * AUDIO_TRIG_MULT_NUM / AUDIO_TRIG_MULT_DEN;
        if (thr < AUDIO_TRIG_MIN_RMS) { thr = AUDIO_TRIG_MIN_RMS; }
        Debug_Printf("[CAP] baseline=%u noise_rms=%u thresh_rms=%lu\r\n",
                     (unsigned)baseline, (unsigned)nrms, (unsigned long)thr);
    }

    /* ② 等语音触发（最多 AUDIO_TRIG_TIMEOUT_MS） */
    if (!WaitForVoiceTrigger())
    {
        /* 超时时把诊断值打出来 —— 一眼看出是"没说话"还是"阈值定太高"：
         *   peak_rms << thresh_rms  → 声音确实不够大（离得远/没说话）
         *   peak_rms 接近 thresh    → 就差一点，把倍数调小或靠近点说
         *   noise_rms 很大          → 环境太吵，阈值被抬高了 */
        audio_trig_diag_t dg;
        audio_get_trigger_diag(&dg);
        Debug_Printf("[CAP] no voice (timeout) noise=%u thresh=%u peak=%u\r\n",
                     (unsigned)dg.noise_rms, (unsigned)dg.thresh_rms,
                     (unsigned)dg.peak_rms);
        strcpy(last_result, "NO_VOICE");
        OLED_UpdateStatus();
        return;
    }
    Debug_SendString("[CAP] voice triggered, capturing 1s ...\r\n");

    /* ③ 采 1 秒真实音频（8000 点 @8kHz） */
    CaptureVoiceWindow();

    /* ④ 本地也算一遍时域特征（用于对比 / 排查） */
    feat = ExtractVoiceFeature();
    Debug_Printf("[CAP] mean=%u min=%u max=%u p2p=%u energy=%lu\r\n",
                 (unsigned)feat.mean, (unsigned)feat.min, (unsigned)feat.max,
                 (unsigned)feat.p2p, (unsigned long)feat.energy);
    Debug_Printf("[CAP] seg=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                 (unsigned long)feat.seg1, (unsigned long)feat.seg2,
                 (unsigned long)feat.seg3, (unsigned long)feat.seg4,
                 (unsigned long)feat.seg5, (unsigned long)feat.seg6,
                 (unsigned long)feat.seg7, (unsigned long)feat.seg8);

    /* ⑤ 压缩并上传给 ESP32-S3 */
    Debug_SendString("[CAP] uploading to ESP32-S3 ...\r\n");
    SendAudioToESP32();
    Debug_SendString("[CAP] done\r\n");
}
/* =============================================================================
 *  Uart3_Diag — USART3 硬件状态诊断（'x' 命令）
 * =============================================================================
 *  为什么需要这个：
 *    "ESP32 收不到"这类问题，通常要么是①F103 根本没发出去，
 *    要么是②发出了但线上/对端有问题。没有示波器时很难区分。
 *    这里直接读 **USART3 的状态寄存器（SR）和控制寄存器（CR1）**：
 *      · CR1 的 TE 位  = 发送器是否使能（没使能就什么都不会发）
 *      · CR1 的 RE 位  = 接收器是否使能
 *      · SR 的 TXE 位  = 发送数据寄存器空（可以写下一个字节）
 *      · SR 的 TC 位   = 发送完成（最后一个字节已移出移位寄存器）
 *      · SR 的 ORE 位  = 溢出错误（接收侧来不及取数据）
 *    如果 **TXE/TC 都正常置位，说明芯片内部发送通路是通的** ⇒
 *    问题在芯片外部（引脚虚焊 / 线 / 对端）。
 *
 *  同时把 GPIOB 的配置也读出来，确认 PB10 处在 **复用推挽（AF_PP）** 模式，
 *  而不是被别的代码改成了普通 IO（那样就发不出去了）。
 * ========================================================================== */
static void Uart3_Diag(void)
{
    Debug_SendString("\r\n=== USART3 HW DIAG ===\r\n");

    /* ---- 发送前 ---- */
    Debug_Printf("[DIAG] before: SR=0x%04X CR1=0x%04X\r\n",
                 (unsigned)USART3->SR, (unsigned)USART3->CR1);
    Debug_Printf("[DIAG]   TE=%u RE=%u TXE=%u TC=%u ORE=%u\r\n",
                 (unsigned)((USART3->CR1 >> 3) & 1U),   /* TE  = bit3 */
                 (unsigned)((USART3->CR1 >> 2) & 1U),   /* RE  = bit2 */
                 (unsigned)((USART3->SR  >> 7) & 1U),   /* TXE = bit7 */
                 (unsigned)((USART3->SR  >> 6) & 1U),   /* TC  = bit6 */
                 (unsigned)((USART3->SR  >> 3) & 1U));  /* ORE = bit3 */

    /* ---- PB10 / PB11 的 GPIO 配置 --------------------------------
     * ⚠ 这里踩过一个坑，必须写清楚：
     *   STM32 的 GPIO 配置分两个寄存器：
     *       CRL → 管 PB0 ~ PB7   （每 4 位一个引脚）
     *       CRH → 管 PB8 ~ PB15  （每 4 位一个引脚）
     *   **PB10/PB11 的配置在 CRH 里，不在 CRL 里！**
     *   我第一版代码读的是 CRL>>8 和 CRL>>12 —— 那其实是 PB2/PB3 的配置，
     *   得出来的 0x4 完全是误读，还差点据此误判"PB10 被配成了浮空输入"。
     *   ⇒ 教训：读寄存器前先确认**这个引脚归哪个寄存器管**。
     *
     * CRH 里：PB10 占 bit8~11，PB11 占 bit12~15。
     * 期望值 0xB = CNF=10(复用推挽) + MODE=11(50MHz)；0x9/0xA 也是复用推挽。
     * CNF=00/01 且 MODE=00 ⇒ 输入模式 ⇒ USART 的 TX 根本连不到引脚。 */
    Debug_Printf("[DIAG] GPIOB CRH=0x%08lX  PB10=0x%X PB11=0x%X\r\n",
                 (unsigned long)GPIOB->CRH,
                 (unsigned)((GPIOB->CRH >> 8) & 0xFU),    /* PB10 */
                 (unsigned)((GPIOB->CRH >> 12) & 0xFU));  /* PB11 */
    Debug_Printf("[DIAG]   (expect PB10 = 0x9/0xA/0xB for AF push-pull)\r\n");
    Debug_Printf("[DIAG] RCC APB1ENR=0x%08lX (USART3EN=%u)\r\n",
                 (unsigned long)RCC->APB1ENR,
                 (unsigned)((RCC->APB1ENR >> 18) & 1U));   /* USART3EN = bit18 */
    Debug_Printf("[DIAG] USART3 BRR=0x%04X (115200 should be 0x0271)\r\n",
                 (unsigned)USART3->BRR);

    /* ---- 发一小段已知数据，看 TC 能不能置位 ----
     * 如果 TC 一直不置位 ⇒ 发送移位寄存器没在动 ⇒ 时钟/配置有问题。
     * 如果能置位 ⇒ 数据确实被推出去了 ⇒ 问题在片外。 */
    {
        const char *probe = "DIAG1234";
        const uint32_t t0 = HAL_GetTick();
        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart3,
                                                (uint8_t *)probe, 8, 100);
        const uint32_t dt = HAL_GetTick() - t0;

        Debug_Printf("[DIAG] send 8 bytes -> %s, took %lums\r\n",
                     (st == HAL_OK) ? "HAL_OK" : "FAIL",
                     (unsigned long)dt);
        Debug_Printf("[DIAG] after:  SR=0x%04X (TXE=%u TC=%u)\r\n",
                     (unsigned)USART3->SR,
                     (unsigned)((USART3->SR >> 7) & 1U),
                     (unsigned)((USART3->SR >> 6) & 1U));

        if ((USART3->SR & (1U << 6)) != 0U) {
            Debug_SendString("[DIAG] TC set -> chip DID push bytes out.\r\n");
            Debug_SendString("[DIAG] => problem is OUTSIDE the chip: "
                             "pin solder / wire / peer / GND\r\n");
        } else {
            Debug_SendString("[DIAG] !! TC never set -> TX path stall in chip\r\n");
        }
    }

    /* ---- 引脚翻转测试 ------------------------------------------------
     *  把 PB10 临时改成**普通推挽输出**，以 5Hz 翻转 1 秒。
     *  目的：把"引脚能不能被驱动"和"USART 通路"彻底分开。
     *    · 若外部（对端/示波器）能看到 PB10 电平在跳 ⇒ 引脚是通的，
     *      那 USART 收不到就另有原因；
     *    · 若连 GPIO 翻转都测不到 ⇒ 引脚到排针之间断路（虚焊）。
     *  ⚠ 测完必须恢复成 USART3 复用模式，否则后面通信全废。 */
    {
        GPIO_InitTypeDef gi = {0};

        Debug_SendString("[DIAG] pin toggle test on PB10 (1s, ~5Hz) ...\r\n");

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
        gi.Pin   = GPIO_PIN_10;
        gi.Mode  = GPIO_MODE_OUTPUT_PP;
        gi.Pull  = GPIO_NOPULL;
        gi.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &gi);

        for (int i = 0; i < 10; i++) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
            HAL_Delay(100);
        }

        Debug_Printf("[DIAG] after toggle: CRH=0x%08lX PB10=0x%X\r\n",
                     (unsigned long)GPIOB->CRH,
                     (unsigned)((GPIOB->CRH >> 8) & 0xFU));

        /* ★ 恢复成 USART3 的复用推挽，否则后续通信全废 ★ */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
        gi.Pin   = GPIO_PIN_10;
        gi.Mode  = GPIO_MODE_AF_PP;
        gi.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &gi);

        Debug_Printf("[DIAG] restored: CRH=0x%08lX PB10=0x%X\r\n",
                     (unsigned long)GPIOB->CRH,
                     (unsigned)((GPIOB->CRH >> 8) & 0xFU));
        Debug_SendString("[DIAG] if you have a scope/LED on PB10, "
                         "it should have blinked ~5 times\r\n");
    }
    Debug_SendString("=== DIAG DONE ===\r\n");
}
/* =============================================================================
 *  Uart3_LoopbackTest — USART3 硬件回环测试（'L' 命令）
 * =============================================================================
 *  为什么需要这个：
 *    现象很矛盾 ——
 *      · F103 寄存器证明"数据发出去了"（TC=1、PB10=AF_PP、引脚可翻转）
 *      · ESP32 回环测试证明"它的 UART 收发正常"
 *      · 但两者连起来就是收不到
 *    要打破僵局，必须**把对端排除掉**，只验证 F103 自己的 USART3 收发通路。
 *
 *  做法（只需一根杜邦线）：
 *    把 F103 的 **PB10(TX) 直接接到 PB11(RX)**（自己接自己），
 *    然后本函数发一串已知数据、再从 RX 读回来。
 *      读回一致 ⇒ PB10/PB11 两个引脚 + USART3 都正常
 *      读不回来 ⇒ 这两个引脚（或 USART3）有硬件问题
 *
 *  ⚠ 做测试前请把接在 PB10/PB11 上的 ESP32 线拔掉，避免互相干扰。
 * ========================================================================== */
static void Uart3_LoopbackTest(void)
{
    static const char PATTERN[] = "F103LOOP-0123456789";
    const uint16_t plen = (uint16_t)(sizeof(PATTERN) - 1U);   /* 不含结尾 '\0' */

    Debug_SendString("\r\n=== USART3 LOOPBACK TEST ===\r\n");
    Debug_SendString("[LOOP] STEP 1: connect a jumper PB10(TX) <-> PB11(RX)\r\n");
    Debug_SendString("[LOOP] (unplug the ESP32 wires first!)\r\n");
    Debug_SendString("[LOOP] starting in 5 s ...\r\n");
    HAL_Delay(5000);

    /* 清空接收缓冲（避免把历史数据当结果）*/
    esp_rx_len = 0;
    memset(esp_rx_buf, 0, sizeof(esp_rx_buf));
    __HAL_UART_CLEAR_OREFLAG(&huart3);

    Debug_Printf("[LOOP] TX %u bytes: %s\r\n", (unsigned)plen, PATTERN);

    /* 用**阻塞发送**：HAL_UART_Transmit 会等到 TC 置位才返回，
     * 这样"发完"和"开始读"之间没有竞态。 */
    if (HAL_UART_Transmit(&huart3, (uint8_t *)PATTERN, plen, 200) != HAL_OK) {
        Debug_SendString("[LOOP] !! transmit failed\r\n");
        return;
    }

    /* 回环是即时的，但中断接收需要时间，给足 300ms */
    HAL_Delay(300);

    Debug_Printf("[LOOP] RX got %u bytes\r\n", (unsigned)esp_rx_len);

    if (esp_rx_len == 0U) {
        Debug_SendString("[LOOP] => FAILED: nothing came back.\r\n");
        Debug_SendString("[LOOP]    check: is the jumper really PB10 <-> PB11 ?\r\n");
        Debug_SendString("[LOOP]    if yes -> PB10/PB11 pins have a HW problem\r\n");
        return;
    }

    Debug_Printf("[LOOP] RX data: %s\r\n", esp_rx_buf);

    if (strcmp(esp_rx_buf, PATTERN) == 0) {
        Debug_SendString("[LOOP] => OK: USART3 TX+RX both work.\r\n");
        Debug_SendString("[LOOP]    => PB10/PB11 pins are electrically fine.\r\n");
        Debug_SendString("[LOOP]    => then the problem is in the wiring to ESP32,"
                         " or the ESP32 side.\r\n");
    } else {
        Debug_SendString("[LOOP] => PARTIAL: got data but does not match.\r\n");
        Debug_SendString("[LOOP]    => baud/framing problem inside USART3.\r\n");
    }
    Debug_SendString("=== LOOPBACK DONE ===\r\n");
}
/* =============================================================================
 *  Audio_VerifyAndAct — 完整的门禁流程（'u' 命令）
 * =============================================================================
 *  流程：
 *    ① 测静音基线 + 噪声底
 *    ② 等语音触发（帧能量 VAD）
 *    ③ 采 1 秒真实音频（8000 点 @8kHz）
 *    ④ 压缩后通过 UART3 发给 ESP32-S3
 *    ⑤ **等待 ESP32 回传判定结果**（PASS/FAIL）
 *    ⑥ PASS → 舵机开锁 + OLED 显示；FAIL → 拒绝；超时 → 报错
 *
 *  ⚠ 第 ⑤ 步需要 **PB11(USART3_RX) ← ESP32 IO6(UART2_TX)** 那根线。
 *    没接的话会一直等到超时（打印 "no verdict"）。
 * ========================================================================== */
#define VERDICT_TIMEOUT_MS   8000U   /* 等 ESP32 判定的最长时间 */

static void Audio_VerifyAndAct(void)
{
    uint16_t baseline;
    VoiceFeature_t feat;

    if (!audio_is_running())
    {
        Debug_SendString("[CAP] audio not running (audio_init failed?)\r\n");
        return;
    }

    Debug_SendString("\r\n=== VOICEPRINT VERIFY (8kHz) ===\r\n");

    /* ① 基线 + 噪声底 */
    baseline = MeasureBaseline();
    {
        const uint16_t nrms = audio_measure_noise_rms();
        uint32_t thr = (uint32_t)nrms * AUDIO_TRIG_MULT_NUM / AUDIO_TRIG_MULT_DEN;
        if (thr < AUDIO_TRIG_MIN_RMS) { thr = AUDIO_TRIG_MIN_RMS; }
        Debug_Printf("[CAP] baseline=%u noise_rms=%u thresh_rms=%lu\r\n",
                     (unsigned)baseline, (unsigned)nrms, (unsigned long)thr);
    }

    /* ② 等语音触发 */
    if (!WaitForVoiceTrigger())
    {
        audio_trig_diag_t dg;
        audio_get_trigger_diag(&dg);
        Debug_Printf("[CAP] no voice (timeout) noise=%u thresh=%u peak=%u\r\n",
                     (unsigned)dg.noise_rms, (unsigned)dg.thresh_rms,
                     (unsigned)dg.peak_rms);
        strcpy(last_result, "NO_VOICE");
        OLED_UpdateStatus();
        return;
    }
    Debug_SendString("[CAP] voice triggered, capturing 1s ...\r\n");

    /* ③ 采 1 秒 */
    CaptureVoiceWindow();

    /* ④ 本地时域特征（调试用） */
    feat = ExtractVoiceFeature();
    Debug_Printf("[CAP] mean=%u min=%u max=%u p2p=%u energy=%lu\r\n",
                 (unsigned)feat.mean, (unsigned)feat.min, (unsigned)feat.max,
                 (unsigned)feat.p2p, (unsigned long)feat.energy);

    /* ⑤ 上传给 ESP32，然后等判定 */
    strcpy(last_result, "VERIFY");
    OLED_UpdateStatus();

    Debug_SendString("[CAP] uploading to ESP32-S3 ...\r\n");
    SendAudioToESP32();

    Debug_SendString("[CAP] waiting for verdict ...\r\n");

    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < VERDICT_TIMEOUT_MS)
    {
        if (uartResultReady())
        {
            UartResult r;
            uartGetResult(&r);

            /* ★ 保存相似度供 OLED 显示 ★
             *   ESP32 回传的 similarity 是"0~1000 对应 0.000~1.000"，
             *   存成千分比后 OLED 显示成 SIM:0.732 这种形式，
             *   能直观说明"为什么这次通过/不通过"。 */
            last_sim_permille = r.similarity;

            Debug_Printf("[RES] verdict=%s confidence=%u sim=%u.%03u "
                         "esp_took=%lums\r\n",
                         (r.verdict == UART_VERDICT_PASS) ? "PASS" : "FAIL",
                         (unsigned)r.confidence,
                         (unsigned)(r.similarity / 1000U),
                         (unsigned)(r.similarity % 1000U),
                         (unsigned long)r.elapsedMs);

            if (r.verdict == UART_VERDICT_PASS)
            {
                strcpy(last_result, "PASS");
                OLED_UpdateStatus();     /* ★ 屏幕显示 *** PASS *** */
                Debug_SendString("[RES] === ACCESS GRANTED ===\r\n");
                Door_Open();
                /* ★ 结果在屏幕上停留 10 秒 ★
                 *   原来只停 3 秒 —— 录演示视频时经常来不及拍清楚，
                 *   观众也看不清屏幕上写的是什么。10 秒足够从容展示。
                 *   ⚠ 这 10 秒是阻塞的，但这正是"门开着"的语义，
                 *     期间不接受新的采集请求是合理的。 */
                HAL_Delay(10000);
                Door_Close();
                Debug_SendString("[RES] back to idle\r\n");
            }
            else
            {
                strcpy(last_result, "FAIL");
                OLED_UpdateStatus();     /* ★ 屏幕显示 --- FAIL --- */
                Debug_SendString("[RES] === ACCESS DENIED ===\r\n");
                /* 拒绝结果同样停留 10 秒，保证演示时看得清 */
                HAL_Delay(10000);
            }
            return;
        }
        HAL_Delay(10);
    }

    /* 超时：多半是回传线没接 */
    strcpy(last_result, "NO_RES");
    OLED_UpdateStatus();
    Debug_Printf("[RES] !! no verdict in %ums\r\n", VERDICT_TIMEOUT_MS);
    Debug_SendString("[RES] check the wire: F103 PB11(RX) <- ESP32 IO6(TX)\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Debug_SendString("\r\n[BOOT] start\r\n");

  /* ★ 门锁指示改用 OLED（不再驱动舵机）★
   *   舵机方案放弃的原因见文件里"舵机相关代码已移除"那段注释。
   *   ⚠ 不再调用 HAL_TIM_PWM_Start(&htim3, ...) —— 如果舵机还插在 PB0 上，
   *     拔掉它即可（没有 PWM 信号时它会轻微嗡响）。 */
  Debug_SendString("[BOOT] door indicator = OLED (no servo)\r\n");

  OLED_Init();
  OLED_Clear();
  Debug_SendString("[BOOT] OLED ok\r\n");

  /* ★ 已从启动流程中移除 ESP8266_HardwareReset() ★
   * 原因：
   *   ① 它内部有 HAL_Delay(200) + HAL_Delay(1500) = **1.7 秒**，
   *      是启动慢的主要原因（现象："前面 boot 检测等好久"）。
   *   ② 它是给 **ESP8266**（AT 指令模块，EN/RST 接 PB8/PB9）做的硬件复位；
   *      而升级后的方案是**用 UART3(PB10/PB11) 直连 ESP32-S3 传音频**，
   *      不需要（也不应该）去复位对端 —— 那会把对端的接收缓冲清掉。
   *   函数本身保留着：敲 'r' 命令仍可手动复位（调试用）。
   *   复位状态变量，避免上层误判"已连接"。 */
  wifi_ready       = 0;
  server_connected = 0;
  pass_streak      = 0;
  Debug_SendString("[BOOT] ESP reset skipped (not needed for UART3 link)\r\n");

  Door_Close();
  Debug_SendString("[BOOT] door closed\r\n");

  /* ★ 关键顺序修正 ★
   * Mic_ReadOnce() 依赖 ADC 处于"软件触发单次转换"模式（用 HAL_ADC_Start +
   * PollForConversion）。而下面的 audio_init() 会把 ADC 改成"TIM2_CH2 外部触发"，
   * 改完之后 Mic_ReadOnce() 的 HAL_ADC_Start 不再产生转换 → PollForConversion
   * 死等 → 板子卡在这里（现象：只打印到 "Door Close" 就没了）。
   * 所以必须**在 audio_init() 之前**读这一次 ADC。 */
  mic_adc_value = Mic_ReadOnce();
  Debug_Printf("[BOOT] mic_adc=%u\r\n", (unsigned)mic_adc_value);

  OLED_UpdateStatus();
  Debug_SendString("[BOOT] oled status ok\r\n");

  ShowHelp();

  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  Debug_SendString("[BOOT] uart it ok\r\n");

  /* ---- 新增：启动 8kHz 音频采集 ----------------------------------------
   * 把 ADC1 从"软件触发单次转换"改成"TIM2_CH2 触发 + DMA 循环"，
   * 采样率从原来的 250Hz 提到 8000Hz（详见 audio_capture.h 的说明）。
   * ⚠ 失败时不再调用 Error_Handler()（那会关中断 + 死循环，板子一声不响
   *   地卡住，串口什么都看不到），而是打印出具体卡在哪一步。
   * ⚠ 这一步之后 ADC 就是外部触发模式了，Mic_ReadOnce() 不再可用。
   *
   * ★ 必须和 AUDIO_INIT_OK 显式比较，不能写 if (audio_init()) ★
   *   因为 AUDIO_INIT_OK == 0，而 C 里"0 视为假"：
   *   if (audio_init()) 会把**成功**判成失败（0 → 走 else 分支）。
   *   这个坑很隐蔽 —— 现象是打印 "init FAILED, rc=0"，而 rc=0 恰恰是成功码。 */
  {
      const audio_init_rc_t rc = audio_init();
      if (rc == AUDIO_INIT_OK)
      {
          Debug_SendString("[AUDIO] 8kHz capture started (TIM2 -> ADC1 -> DMA)\r\n");
      }
      else
      {
          Debug_Printf("[AUDIO] init FAILED, rc=%d "
                       "(1=TIM2 2=DMA 3=ADC_Init 4=ADC_Channel 5=StartDMA)\r\n",
                       (int)rc);
      }
  }
  Debug_SendString("[BOOT] all init done\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- 串口命令分发 --------------------------------------------------
     * ⚠ 原来这里是空的：中断把 uart1_cmd_ready 置了 1，但主循环从不检查它，
     *   所以敲任何命令都没有反应。这里补上分发逻辑。 */
    if (uart1_cmd_ready)
    {
        /* ★ 大小写归一化（关键修复）★
         * 原来 `switch (c)` 直接用原始字符，而 case 标签是小写（'a'、't'、'x'…），
         * 所以敲**大写**字母一律落到 default，报 "unknown command"。
         * 用户很自然会敲大写，这个坑很难自己发现。
         * 做法：统一转成大写，case 标签也全部用大写 —— 大小写都能用。 */
        const char c = (char)toupper((int)uart1_cmd_byte);
        uart1_cmd_ready = 0;

        switch (c)
        {
        case 'A':
            ESP8266_SendString("AT\r\n");
            Debug_SendString("[CMD] ESP8266 AT\r\n");
            break;

        case 'R':
            ESP8266_HardwareReset();
            Debug_SendString("[CMD] ESP8266 reset\r\n");
            break;

        case 'O':
            Door_Open();
            Debug_SendString("[CMD] Door OPEN\r\n");
            break;

        case 'C':
            Door_Close();
            Debug_SendString("[CMD] Door CLOSE\r\n");
            break;

        case 'W':
            Debug_SendString("[CMD] WiFi connect\r\n");
            ESP_InitWiFi();
            break;

        case 'S':
            Debug_SendString("[CMD] Server connect\r\n");
            ESP_ConnectServer();
            break;

        case 'V':
            Debug_SendString("[CMD] Voice verify\r\n");
            ESP_CloudVerifyOnce();
            break;

        case 'T':                       /* ★ 音频采集自检（不带触发/上传）*/
            Audio_SelfTest();
            break;

        case 'U':                       /* ★ 完整门禁：采集+上传+等判定+开锁 */
            Audio_VerifyAndAct();
            break;

        case 'P':                       /* 只上传、不等判定（链路测试用）*/
            Audio_CaptureAndUpload();
            break;

        case 'K':                       /* ★ 声学标定（测静音/说话 RMS）*/
            Audio_Calibrate();
            break;

        case 'X':                       /* ★ USART3 硬件状态诊断 */
            Uart3_Diag();
            break;

        case 'D':                       /* ★ OLED / I2C 总线诊断 */
            I2C_Diag();
            break;

        case 'L':                       /* ★ USART3 回环测试（PB10<->PB11 短接）*/
            Uart3_LoopbackTest();
            break;

        case 'H':
        case '?':
            ShowHelp();
            break;

        default:
            Debug_Printf("[CMD] unknown '%c' (press h for help)\r\n", c);
            break;
        }
    }

    /* ★ OLED 显示看门狗（每 2 秒一次）★
     * ----------------------------------------------------------------
     *  为什么必须有这个（真实踩坑记录）：
     *    端到端测试时屏幕突然全黑，但 I2C 统计显示 **err=0**（1079 次传输
     *    零错误），按复位键屏幕又正常 —— 说明**通信链路没问题**，
     *    是 SSD1306 自己"掉显示"了：音频采集（ADC+DMA 全速）+ ESP32 高速
     *    串口收发让电流骤增、电压瞬间跌落，把 SSD1306 内部的 Display ON
     *    寄存器复位了；而 STM32 有电容撑着，主程序毫无察觉继续发数据。
     *
     *  对策：周期性重发"配置 + 开显示"命令。
     *    · 屏幕正常时：重发同样的命令**无副作用**（不会闪屏，因为没清 RAM）
     *    · 屏幕掉了时：立刻被重新点亮
     *    开销约 3ms / 2s，可以忽略。
     * ---------------------------------------------------------------- */
    if ((HAL_GetTick() - last_oled_tick) >= 2000U)
    {
        last_oled_tick = HAL_GetTick();
        SSD1306_Reinit();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_cmd_byte = uart1_rx_byte;
        uart1_cmd_ready = 1;
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        if (esp_rx_len < (ESP_RX_BUF_LEN - 1))
        {
            esp_rx_buf[esp_rx_len++] = (char)uart3_rx_byte;
            esp_rx_buf[esp_rx_len] = '\0';
        }

        /* ★ 同时喂给二进制协议解析器 ★
         * 为什么要"两路并行"：
         *   · esp_rx_buf 是给**文本 AT 指令时代**留的（原来接 ESP8266 的用法）
         *   · uartRxFeed 是给**双片二进制协议**用的（ESP32-S3 回传 PASS/FAIL）
         * 两者互不干扰：文本内容不会凑出合法 CRC；二进制内容进 esp_rx_buf
         * 后没人读它，无害。这样不用删掉旧逻辑，风险最小。 */
        uartRxFeed(uart3_rx_byte);

        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
