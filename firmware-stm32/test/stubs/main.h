/**
 * @file    main.h  （PC 端语法检查用桩文件，**不是真的 HAL**）
 * @brief   仅用于在 PC 上用 gcc -fsyntax-only 检查 App 目录下各 .c 文件的语法与类型错误。
 *          真正的编译由 CubeMX 生成的工程完成（真实 main.h 由 CubeMX 覆盖/生成）。
 *          用法见 test/build_and_run.ps1 -SyntaxOnly
 */
#ifndef STUB_MAIN_H
#define STUB_MAIN_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct { uint32_t SQR1; } ADC_TypeDef;
typedef struct { uint32_t CFSR; uint32_t HFSR; } SCB_Type;
extern ADC_TypeDef *const ADC1;
extern SCB_Type *const SCB;

#define __HAL_ADC_DISABLE(h)  do { (void)(h); } while (0)
#define __HAL_ADC_ENABLE(h)   do { (void)(h); } while (0)

typedef struct {
    uint32_t ClockPrescaler;
    uint32_t ScanConvMode;
    uint32_t ContinuousConvMode;
    uint32_t DMAContinuousRequests;
    uint32_t NbrOfConversion;
} ADC_InitTypeDef;
typedef struct {
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
    uint32_t Offset;
} ADC_ChannelConfTypeDef;
typedef struct {
    uint32_t Channel;
    uint32_t Direction;
    uint32_t PeriphInc;
    uint32_t MemInc;
    uint32_t PeriphDataAlignment;
    uint32_t MemDataAlignment;
    uint32_t Mode;
    uint32_t Priority;
    uint32_t FIFOMode;
} DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { void *Instance; void *DMA_Handle; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
typedef struct { void *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } IWDG_HandleTypeDef;
typedef struct { uint32_t dummy; } GPIO_TypeDef;

extern GPIO_TypeDef *const GPIOA;
extern GPIO_TypeDef *const GPIOB;
extern GPIO_TypeDef *const GPIOC;

#define GPIO_PIN_1   (1u << 1)
#define GPIO_PIN_2   (1u << 2)
#define GPIO_PIN_4   (1u << 4)
#define GPIO_PIN_5   (1u << 5)
#define GPIO_PIN_10  (1u << 10)
#define GPIO_PIN_11  (1u << 11)
#define GPIO_PIN_12  (1u << 12)

#define I2C_MEMADD_SIZE_8BIT  1u
#define DMA2_Stream0          0u
#define DMA_CHANNEL_0         0u
#define DMA_PERIPH_TO_MEMORY  0u
#define DMA_PINC_DISABLE      0u
#define DMA_MINC_ENABLE       0u
#define DMA_PDATAALIGN_HALFWORD 0u
#define DMA_MDATAALIGN_HALFWORD 0u
#define DMA_CIRCULAR          0u
#define DMA_PRIORITY_LOW      0u
#define DMA_FIFOMODE_DISABLE  0u
#define DMA2_Stream0_IRQn     11

#define ENABLE                1u
#define DISABLE               0u
#define ADC_CHANNEL_8         8u
#define ADC_CHANNEL_9         9u
#define ADC_CHANNEL_10        10u
#define ADC_CHANNEL_11        11u
#define ADC_CHANNEL_12        12u
#define ADC_CHANNEL_13        13u
#define ADC_CHANNEL_14        14u
#define ADC_CHANNEL_15        15u
#define ADC_SAMPLETIME_84CYCLES 0u
#define ADC_CLOCK_SYNC_PCLK_DIV4 0u
#define TIM_CHANNEL_1         1u
#define TIM_CHANNEL_2         2u
#define TIM_CHANNEL_3         3u
#define TIM_CHANNEL_4         4u
#define DMA_IT_HT             0x10u
#define UART_FLAG_TC          0x40u
#define UART_IT_TC            0x20u
#define RESET                 0
#define SET                   1
#define HAL_UART_STATE_READY  0u

GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void              HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t len);
void              HAL_NVIC_DisableIRQ(int32_t irqn);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t len);
uint32_t          HAL_UART_GetState(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                                    uint16_t reg_size, uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg,
                                   uint16_t reg_size, uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr, uint32_t trials, uint32_t timeout);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buf, uint32_t len);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *cfg);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *h);
uint32_t          HAL_GetTick(void);
void              NVIC_SystemReset(void);

#define __HAL_TIM_SET_COMPARE(h, ch, v)  do { (void)(h); (void)(ch); (void)(v); } while (0)
#define __HAL_RCC_DMA2_CLK_ENABLE()      do { } while (0)
#define __HAL_LINKDMA(handle, field, dma) do { (handle)->field = &(dma); } while (0)
#define __HAL_DMA_DISABLE_IT(h, it)      do { (void)(h); (void)(it); } while (0)
#define __HAL_UART_GET_FLAG(h, f)        (0u)
#define __HAL_UART_CLEAR_FLAG(h, f)      do { (void)(h); (void)(f); } while (0)

#endif /* STUB_MAIN_H */
