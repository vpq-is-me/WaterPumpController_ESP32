#ifndef DISPLAY_AND_ADC_H
#define DISPLAY_AND_ADC_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

void DisplayInit(EventGroupHandle_t events);
void DisplaySetPressure(uint32_t new_press);
void DisplaySetCounter(uint32_t new_counter);
int32_t ADCGetResult(void);

#endif