/* board.h - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define LED_ON  1
#define LED_OFF 0
#define PP_REL_ON  1
#define PP_REL_OFF 0

#define LED_B                   GPIO_NUM_2
#define BOARD_BUTTON_PIN        GPIO_NUM_0
#define COUNTER_IN_PIN          GPIO_NUM_17
#define PUMP_POWER_IN_PIN       GPIO_NUM_23
#define PUMP_RELAY_OUT_PIN      GPIO_NUM_4  //set to 1 for close contacts
#define PUMP_TRIAC_OUT_PIN      GPIO_NUM_16 //set to 0 for triac became conducting
#define BYPASS_SW_POS_PIN       GPIO_NUM_19
// #define I2C_MASTER_SCL_IO 26        /*!< gpio number for I2C master clock */
// #define I2C_MASTER_SDA_IO 25        /*!< gpio number for I2C master data  */

void LED_operation(uint8_t onoff);
void LED_toggle();
void PumpRelay(uint8_t onoff);
uint32_t IsPumpRun(void);

void board_init(void);

void DigInInit(EventGroupHandle_t events);
void DigInSetTankData(uint16_t vol,int32_t pres);
uint32_t IsBypassSwitchOn(void);
uint8_t IsButtonPressed(void);

#define MAX_PUMP_RUN_TIME_MNT   2
#define MAX_PUMP_RUN_TIME_MS   (MAX_PUMP_RUN_TIME_MNT*60*1000)
#define MIN_PUMP_STOPPED_TIME_SEC 10
#define MIN_PUMP_STOPPED_TIME_MS (MIN_PUMP_STOPPED_TIME_SEC*1000)

volatile extern uint32_t pump_last_run_time_ms;
volatile extern float volume_at_WFpulse;
volatile extern TickType_t time_from_start_at_WFpulse;
volatile extern TickType_t time_pp_on_max;
volatile extern TickType_t time_pp_on_min;

/* Pressure transmitter measuring range*/
#define PRES_TRANS_RANGE_bar  6.0   
#define ADC_REF_VOLT          2.048 //check adc PGA settings
#define ADC_MAX_CODE          0x7fff
//Input is differential signal between sensor 4-20mA voltage drop on Rshunt 
//and offset voltage. Ofset voltage tuned to be equal of voltage on Rshunt at 4mA sensor current.
//In this case we measure voltage drop on Rshunt concerned to current range 0-16mA
#define RSHUNT 130.0
#define PRES_ADC_MAX_CODE     ((0.016*RSHUNT*ADC_MAX_CODE)/ADC_REF_VOLT)

#define Pressure2ADCcode(pr)  (int32_t)(PRES_ADC_MAX_CODE*((pr)/PRES_TRANS_RANGE_bar))
#define ADCcode2Pressure(cd)  (PRES_TRANS_RANGE_bar*((float)(cd))/PRES_ADC_MAX_CODE)
#define ADC_1BAR (int32_t)(PRES_ADC_MAX_CODE*(1.0/PRES_TRANS_RANGE_bar))
#endif
