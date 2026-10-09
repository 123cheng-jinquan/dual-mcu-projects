/**
 * @file    task.h  （PC 端语法检查用桩文件）
 */
#ifndef STUB_TASK_H
#define STUB_TASK_H

#include "FreeRTOS.h"

typedef void (*TaskFunction_t)(void *);

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint16_t stack_depth,
                       void *arg, UBaseType_t prio, TaskHandle_t *created);
TickType_t xTaskGetTickCount(void);
void       vTaskDelayUntil(TickType_t *prev_wake, TickType_t increment);
void       vTaskDelay(TickType_t ticks);
void       taskYIELD(void);

#endif /* STUB_TASK_H */
