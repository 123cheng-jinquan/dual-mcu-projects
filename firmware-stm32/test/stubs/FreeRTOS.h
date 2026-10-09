/**
 * @file    FreeRTOS.h  （PC 端语法检查用桩文件，不是真的 FreeRTOS）
 */
#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H

#include <stdint.h>
#include <stddef.h>

typedef uint32_t       TickType_t;
typedef long           BaseType_t;
typedef unsigned long  UBaseType_t;
typedef void          *QueueHandle_t;
typedef void          *TaskHandle_t;
typedef void          *SemaphoreHandle_t;

#define pdPASS          1
#define pdFAIL          0
#define pdTRUE          1
#define pdFALSE         0
#define portMAX_DELAY   0xFFFFFFFFu
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define portYIELD_FROM_ISR(x) do { (void)(x); } while (0)

#endif /* STUB_FREERTOS_H */
