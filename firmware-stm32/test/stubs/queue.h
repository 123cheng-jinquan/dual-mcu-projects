/**
 * @file    queue.h  （PC 端语法检查用桩文件）
 */
#ifndef STUB_QUEUE_H
#define STUB_QUEUE_H

#include "FreeRTOS.h"

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
BaseType_t    xQueueReceive(QueueHandle_t q, void *item, TickType_t wait);
BaseType_t    xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken);

#endif /* STUB_QUEUE_H */
