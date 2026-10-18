/**
 * @file    app_main.h
 * @brief   应用层入口：CubeMX 生成的句柄 → 业务初始化 + 任务创建
 *
 * 用法（见 CUBEMX-SETUP.md §4）：
 *   1) 在 Core/Src/main.c 的 USER CODE BEGIN Includes 里 #include "app_main.h"
 *   2) 在 USER CODE BEGIN 2 里调用 app_init(&huart1, &huart2, &hadc1, &hi2c1, &htim1, &htim2, &htim3);
 *   3) 在 USER CODE BEGIN RTOS_THREADS 里调用 app_start_tasks();
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "main.h"   /* CubeMX 生成的句柄类型 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 业务初始化（在调度器启动前调用）
 * @param huart_dbg 调试串口（USART1 / CH340），用于 printf
 * @param huart_mb  Modbus 串口（USART2）
 * @param hadc      ADC1（8 通道扫描 + DMA）
 * @param hi2c      I2C1（传感器总线）
 * @param htim1     TIM1（三色灯 R）
 * @param htim2     TIM2（舵机 50Hz）
 * @param htim3     TIM3（风扇 / 电机 / RGB G,B）
 */
void app_init(UART_HandleTypeDef *huart_dbg,
              UART_HandleTypeDef *huart_mb,
              ADC_HandleTypeDef  *hadc,
              I2C_HandleTypeDef  *hi2c,
              TIM_HandleTypeDef  *htim1,
              TIM_HandleTypeDef  *htim2,
              TIM_HandleTypeDef  *htim3);

/** 创建全部业务任务（在 MX_FREERTOS_Init 里、调度器启动前调用） */
void app_start_tasks(void);

/**
 * @brief USART2 中断钩子（**可选**）
 * @note  本工程的 485 发送采用"任务内等待 TC 后切 DE"的写法（见 app_main.c），
 *        因此**不需要**改 stm32f4xx_it.c。若将来想改成纯中断切 DE，可在
 *        stm32f4xx_it.c 的 USART2_IRQHandler 里、调用 HAL_UART_IRQHandler 之前
 *        调用本函数，并在其中处理 UART_FLAG_TC。
 */
void app_uart2_irq_hook(void);

#ifdef __cplusplus
}
#endif
#endif /* APP_MAIN_H */
