#ifndef SEMPHR_H
#define SEMPHR_H

#include "FreeRTOS.h"

/* Host stub for the FreeRTOS mutex API. Functions here; macros in the real
 * header, with the same signatures. */

typedef struct QueueDefinition *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
void vSemaphoreDelete(SemaphoreHandle_t sem);

#endif
