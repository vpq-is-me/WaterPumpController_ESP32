#ifndef LOOP_H
#define LOOP_H


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main.h"

#define LOOP_STACK_SIZE 4096

void loop_task(void* arg);


//#define WP_ALARM_TANK_VOLUME_LOW        (0x0001ul << 0)
//#define WP_ALARM_TANK_VOLUME_LOW_LOW    (0x0001ul << 1)
#define WP_ALARM_CHECKVALVE_LEAK        (0x0001ul << 2)
#define WP_ALARM_PUMP_CAPACITY_LOW      (0x0001ul << 3)
#define WP_ALARM_PUMP_CAPACITY_LOW_LOW  (0x0001ul << 4)
#define WP_ALARM_BYPASS_SW_ON           (0x0001ul << 5)
#define WP_ALARM_PUMP_LONG_RUN          (0x0001ul << 6)
#define WP_ALARM_FREQUENT_START         (0x0001ul << 7)
#define WP_ALARM_FREQUENT_START_HIGH    (0x0001ul << 8)
#define WP_ALARM_SEPTIC_REQ_SDWN        (0x0001ul << 9)
#define WP_ALARM_SEPTIC_NOT_SEND        (0x0001ul << 10)

#define EVENT_NEW_COUNT        (0x01<<0)
#define EVENT_PUMP_START       (0x01<<1)
#define EVENT_PUMP_STOP        (0x01<<2)
#define EVENT_PUMP_RUN_LONG    (0x01<<3)
#define EVENT_PUMP_TOO_FREQ    (0x01<<4)
#define EVENT_NEW_PRESSURE_VAL (0x01<<5)

typedef enum tWP_param_i_en{WP_PARAM_I_COUNTER,
                            WP_PARAM_I_ALARMS,
                            WP_PARAM_I_PRESS_MAX_STOP,
                            WP_PARAM_I_PRESS_MIN_START,
                            WP_PARAM_I_AIR_PRESS,
                            WP_PARAM_I_TK_GROSS_VOL,
                            WP_PARAM_I_SHUTDOWN                            
                            }WP_param_id;
                            
int32_t WPGetParameter(WP_param_id id);

int32_t WPSetParameter(WP_param_id id, int32_t val);
void WPSetSepticState(uint8_t dat);

#endif