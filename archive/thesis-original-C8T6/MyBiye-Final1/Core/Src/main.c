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
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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

/* 采样参数：8段，总长约 880ms */
#define SAMPLE_COUNT              220U
#define SAMPLE_INTERVAL_MS        4U

/* 语音触发参数 */
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

uint32_t last_adc_tick = 0;
uint32_t last_oled_tick = 0;

uint16_t adc_samples[SAMPLE_COUNT];
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

static void Servo_SetAngle(uint8_t angle);
static void Door_Open(void);
static void Door_Close(void);

static uint16_t Mic_ReadOnce(void);
static uint16_t MeasureBaseline(void);
static uint8_t WaitForVoiceTrigger(uint16_t baseline);
static void CaptureVoiceWindow(void);
static VoiceFeature_t ExtractVoiceFeature(void);

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

static void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse_us;

    if (angle > 180U)
    {
        angle = 180U;
    }

    pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180U;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us);
}

static void Door_Open(void)
{
    Servo_SetAngle(DOOR_OPEN_ANGLE);
    door_is_open = 1;
    OLED_UpdateStatus();
    Debug_SendString("Door Open\r\n");
}

static void Door_Close(void)
{
    Servo_SetAngle(DOOR_CLOSE_ANGLE);
    door_is_open = 0;
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

static uint16_t MeasureBaseline(void)
{
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < TRIGGER_BASELINE_COUNT; i++)
    {
        sum += Mic_ReadOnce();
        HAL_Delay(2);
    }

    return (uint16_t)(sum / TRIGGER_BASELINE_COUNT);
}

static uint8_t WaitForVoiceTrigger(uint16_t baseline)
{
    uint32_t start = HAL_GetTick();
    uint8_t hit = 0;

    strcpy(last_result, "WAIT");
    OLED_UpdateStatus();

    while ((HAL_GetTick() - start) < TRIGGER_TIMEOUT_MS)
    {
        uint16_t s = Mic_ReadOnce();
        uint16_t diff = (s > baseline) ? (s - baseline) : (baseline - s);

        if (diff > TRIGGER_DELTA)
        {
            hit++;
            if (hit >= TRIGGER_HITS)
            {
                return 1;
            }
        }
        else
        {
            hit = 0;
        }

        HAL_Delay(2);
    }

    return 0;
}

static void CaptureVoiceWindow(void)
{
    uint16_t i;

    strcpy(last_result, "CAPT");
    OLED_UpdateStatus();

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_samples[i] = Mic_ReadOnce();
        HAL_Delay(SAMPLE_INTERVAL_MS);
    }
}

static VoiceFeature_t ExtractVoiceFeature(void)
{
    VoiceFeature_t f;
    uint32_t sum = 0;
    uint32_t energy_sum = 0;
    uint16_t i;
    uint16_t seg_size;

    f.min = 4095U;
    f.max = 0U;

    f.seg1 = 0U; f.seg2 = 0U; f.seg3 = 0U; f.seg4 = 0U;
    f.seg5 = 0U; f.seg6 = 0U; f.seg7 = 0U; f.seg8 = 0U;

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        sum += adc_samples[i];

        if (adc_samples[i] < f.min) f.min = adc_samples[i];
        if (adc_samples[i] > f.max) f.max = adc_samples[i];
    }

    f.mean = (uint16_t)(sum / SAMPLE_COUNT);
    f.p2p = f.max - f.min;

    seg_size = SAMPLE_COUNT / 8U;

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        uint32_t diff;

        if (adc_samples[i] >= f.mean)
            diff = adc_samples[i] - f.mean;
        else
            diff = f.mean - adc_samples[i];

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

    if (!WaitForVoiceTrigger(baseline))
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

static void OLED_UpdateStatus(void)
{
    char line2[17];
    char line3[17];
    char line4[17];

    OLED_Clear();
    OLED_ShowString(1, 1, "Voice Lock V2");

    snprintf(line2, sizeof(line2), "D:%s W:%s",
             door_is_open ? "OPEN " : "CLOSE",
             wifi_ready ? "OK" : "NO");
    OLED_ShowString(2, 1, line2);

    snprintf(line3, sizeof(line3), "S:%s A:%4u",
             server_connected ? "OK" : "NO",
             mic_adc_value);
    OLED_ShowString(3, 1, line3);

    snprintf(line4, sizeof(line4), "R:%s", last_result);
    OLED_ShowString(4, 1, line4);
}

static void ShowHelp(void)
{
    Debug_SendString("\r\n========================\r\n");
    Debug_SendString("System Ready V2\r\n");
    Debug_SendString("a -> ESP8266 AT\r\n");
    Debug_SendString("r -> ESP8266 Reset\r\n");
    Debug_SendString("o -> Door Open\r\n");
    Debug_SendString("c -> Door Close\r\n");
    Debug_SendString("w -> Connect WiFi\r\n");
    Debug_SendString("s -> Connect Server\r\n");
    Debug_SendString("v -> Trigger + Send 8Seg Feature\r\n");
    Debug_SendString("Two PASS needed to OPEN\r\n");
    Debug_SendString("h -> Help\r\n");
    Debug_SendString("========================\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t cmd = 0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  OLED_Init();
  OLED_Clear();

  ESP8266_HardwareReset();

  Door_Close();
  mic_adc_value = Mic_ReadOnce();
  OLED_UpdateStatus();

  ShowHelp();

  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  /* USER CODE END 2 */

  while (1)
  {
    if (HAL_GetTick() - last_adc_tick >= 50U)
    {
        last_adc_tick = HAL_GetTick();
        mic_adc_value = Mic_ReadOnce();
    }

    if (HAL_GetTick() - last_oled_tick >= 300U)
    {
        last_oled_tick = HAL_GetTick();
        OLED_UpdateStatus();
    }

    if (uart1_cmd_ready)
    {
        __disable_irq();
        cmd = uart1_cmd_byte;
        uart1_cmd_ready = 0;
        __enable_irq();

        Debug_Printf("\r\n[CMD HEX:0x%02X DEC:%d]\r\n", cmd, cmd);

        if (cmd == '\r' || cmd == '\n' || cmd == ' ')
        {
            Debug_SendString("Ignore CR/LF/SPACE\r\n");
            continue;
        }

        if (cmd >= 'A' && cmd <= 'Z')
        {
            cmd = (uint8_t)(cmd - 'A' + 'a');
            Debug_Printf("To lower: %c\r\n", cmd);
        }

        if (cmd == 'o')
        {
            pass_streak = 0;
            strcpy(last_result, "MANUAL");
            OLED_UpdateStatus();
            Door_Open();
        }
        else if (cmd == 'c')
        {
            pass_streak = 0;
            strcpy(last_result, "MANUAL");
            OLED_UpdateStatus();
            Door_Close();
        }
        else if (cmd == 'a')
        {
            ESP_SendCmdWait("AT\r\n", 1500, "OK", NULL, NULL);
        }
        else if (cmd == 'r')
        {
            Debug_SendString("Reset ESP8266...\r\n");
            ESP8266_HardwareReset();
        }
        else if (cmd == 'w')
        {
            ESP_InitWiFi();
        }
        else if (cmd == 's')
        {
            ESP_ConnectServer();
        }
        else if (cmd == 'v')
        {
            ESP_CloudVerifyOnce();
        }
        else if (cmd == 'h')
        {
            ShowHelp();
        }
        else
        {
            Debug_SendString("Unknown cmd\r\n");
        }
    }

    HAL_Delay(10);
  }
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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif