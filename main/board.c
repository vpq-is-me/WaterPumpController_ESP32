/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "loop.h"
#include "display_adc.h"
#include <math.h>
// #include "iot_button.h"



#define TAG "BOARD"
#define TIMER_GRANUL_MS 10
//**********************************************************************************
void LED_operation(uint8_t onoff){
    gpio_set_level(LED_B, onoff);
}
void LED_toggle(){
    gpio_set_level(LED_B,!gpio_get_level(LED_B));
}

static void board_led_init(void)
{
    gpio_reset_pin(LED_B);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_B, LED_OFF);
}
void board_init(void) {
    board_led_init();
    gpio_reset_pin(BOARD_BUTTON_PIN);
    gpio_set_direction(BOARD_BUTTON_PIN, GPIO_MODE_INPUT);
    // board_button_init();
}

//****************************************
// #define BUTTON_IO_NUM 0
// #define BUTTON_ACTIVE_LEVEL 0

// //void vendor_publish_message(void);

// static void button_tap_cb(void *arg) {
//     //    print_vnd_data();
//  //   vendor_publish_message();
// }

// static void board_button_init(void) {
//     button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
//     if (btn_handle) {
//         iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
//     }
// }
static uint8_t button_state=0xff;
static volatile uint8_t button_was_pressed=0;
uint8_t IsButtonPressed(void){
    uint8_t res=button_was_pressed;
    if(res){
        button_was_pressed=0;
    }
    return res;
}
//****************************************
typedef struct{
    uint8_t counter_in_states; //shift register of state of flow-counter input 
    uint8_t pump_power_in_states; //shift register of state of pump power input 
    uint32_t pump_on_timer;//pump working time from pump start, for long run check,in timer granuality 
    uint32_t pump_off_timer;//pump not working time from last stop, for frequent restart check,in timer granuality
    uint8_t pump_run_fg;
    TickType_t pump_on_acc;//accumulated timer of working time from last flow-counter pulse
    TickType_t pump_last_start_2acc;//time of last start of pump, used for accumulation of working time from one flowcounter pulse to another. It is reseted at every pulse
    TickType_t pump_last_start_pure;//time of last start of pump, used to measures longest and shortest pump working time
    TickType_t time_pp_on_max;
    TickType_t time_pp_on_min;
}tDigIn_st;
static tDigIn_st dig_ins={
    .counter_in_states=0xff,
    .pump_power_in_states=0xff,
    .pump_on_acc=0,
    .time_pp_on_max=0,
    .time_pp_on_min=portMAX_DELAY,
};
//because it must be enough long time between flow-counter pulses 
//it is not required to protectect 'pump_last_run_time_ms' and 'press_val_at_cnt_pulse' by any interprocess sharing facilities 
//AND read/write 32bit value supposed to be atomic  
volatile uint32_t pump_last_run_time_ms=0; //pump working time sinse last flow-counter pulse, used for share
volatile float volume_at_WFpulse=0;
volatile TickType_t time_from_start_at_WFpulse=0;
volatile TickType_t time_pp_on_max;
volatile TickType_t time_pp_on_min;


static uint8_t relay_onoff_cmd=PP_REL_OFF;
static int32_t pump_is_on=0;

void DigInLoop(void* tmr) ;
static void RelaySwitchFSM(void);
float TankVolumeCalculation(int32_t pres);
static EventGroupHandle_t input_events;
//**********************************************************************************
void DigInInit(EventGroupHandle_t events){
    //pump relay control pin
    gpio_reset_pin(PUMP_RELAY_OUT_PIN);
    gpio_set_level(PUMP_RELAY_OUT_PIN, PP_REL_OFF);
    gpio_set_direction(PUMP_RELAY_OUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_RELAY_OUT_PIN, PP_REL_OFF);
    gpio_reset_pin(PUMP_TRIAC_OUT_PIN);
    gpio_set_level(PUMP_TRIAC_OUT_PIN, 1);
    gpio_set_direction(PUMP_TRIAC_OUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_TRIAC_OUT_PIN, 1);
    //inputs 
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL<<COUNTER_IN_PIN) | (1ULL<<PUMP_POWER_IN_PIN) | (1ULL<<BYPASS_SW_POS_PIN);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_conf);

    input_events=events;
    TimerHandle_t tmr=xTimerCreate("dig_in_tmr",pdMS_TO_TICKS(TIMER_GRANUL_MS),pdTRUE,NULL,DigInLoop);
    xTimerStart(tmr,0);
    dig_ins.pump_off_timer=0xffffffff;
}
void DigInLoop(void* tmr) {
    float vol;
    RelaySwitchFSM();
    vol=TankVolumeCalculation(ADCGetResult());
    dig_ins.counter_in_states = (dig_ins.counter_in_states << 1) | gpio_get_level(COUNTER_IN_PIN);
    dig_ins.pump_power_in_states = (dig_ins.pump_power_in_states << 1) | gpio_get_level(PUMP_POWER_IN_PIN);
    button_state = (button_state << 1) | gpio_get_level(BOARD_BUTTON_PIN);
    if((button_state&0x07)==0x04)button_was_pressed=1;
    if ((dig_ins.counter_in_states & 0x07) == 0x03) {  // rising front of next '10 liters' pulse
        if(pump_is_on/*relay_onoff_cmd==PP_REL_ON*/){
           dig_ins.pump_on_acc+=xTaskGetTickCount()-dig_ins.pump_last_start_2acc;
           time_from_start_at_WFpulse=pdTICKS_TO_MS(xTaskGetTickCount()-dig_ins.pump_last_start_pure);
           if(!time_from_start_at_WFpulse)time_from_start_at_WFpulse=1;
        }else{
            time_from_start_at_WFpulse=0;
        }
        dig_ins.pump_last_start_2acc=xTaskGetTickCount();//even if it is in OFF noting wrong will happen
        pump_last_run_time_ms=pdTICKS_TO_MS(dig_ins.pump_on_acc);        
        dig_ins.pump_on_acc=0;
        volume_at_WFpulse=vol;
        time_pp_on_max=pdTICKS_TO_MS(dig_ins.time_pp_on_max);
        time_pp_on_min=pdTICKS_TO_MS(dig_ins.time_pp_on_min);
        dig_ins.time_pp_on_max=0;
        dig_ins.time_pp_on_min=portMAX_DELAY;
        xEventGroupSetBits(input_events, EVENT_NEW_COUNT);
    }
    uint8_t p_st=dig_ins.pump_power_in_states & 0x07;//take last 3 input states
    if(p_st==0x04){//pressure switch just connect power to pump
        dig_ins.pump_on_timer=2;//plus 2 ticks from starting antibounce protection
        xEventGroupSetBits(input_events, EVENT_PUMP_START);
        if(dig_ins.pump_off_timer<(MIN_PUMP_STOPPED_TIME_MS/TIMER_GRANUL_MS)){//pump start too often!
            xEventGroupSetBits(input_events,EVENT_PUMP_TOO_FREQ);
        }
        dig_ins.pump_run_fg=1;
    }else if(p_st==0x00){//pump continue working 
        dig_ins.pump_on_timer++;
        if(dig_ins.pump_on_timer==(MAX_PUMP_RUN_TIME_MS/TIMER_GRANUL_MS)){//check if pump work too long without stopping
            xEventGroupSetBits(input_events, EVENT_PUMP_RUN_LONG);
        }
        dig_ins.pump_run_fg=1;
    }else if(p_st==0x03){//pump just stopped (by pressure switch)
        xEventGroupSetBits(input_events, EVENT_PUMP_STOP);
        dig_ins.pump_off_timer=0;
        dig_ins.pump_run_fg=0;
    }else if(p_st==0x07){//pump stopped, mantain pump off timeout
        dig_ins.pump_run_fg=0;
        if(dig_ins.pump_off_timer<(MIN_PUMP_STOPPED_TIME_MS/TIMER_GRANUL_MS+1))//to avoid overlap timer 
            dig_ins.pump_off_timer++;
    }//else -> other posible combinations are assumed as contact bounce 
}

uint32_t IsBypassSwitchOn(void){
    return !gpio_get_level(BYPASS_SW_POS_PIN);
}
uint32_t IsPumpRun(void){
    return dig_ins.pump_run_fg;
}

//**********************************************************************************
typedef enum {REL_OFF, REL_TRIAC_CLS_ON,REL_RELAY_CLS_ON,
              REL_ON,  REL_TRIAC_OPN_ON,REL_RELAY_OPN_OFF}pump_relay_state_en;
void PumpRelay(uint8_t onoff){
    relay_onoff_cmd=onoff;
}
#define RELAYSWITCHMARGINE (50/TIMER_GRANUL_MS)
static void RelaySwitchFSM(void){
    static pump_relay_state_en state = REL_OFF;
    static TickType_t tout;
    switch (state) {
        case REL_OFF:
            if (relay_onoff_cmd == PP_REL_OFF) {
                break;
            } else {
                tout = xTaskGetTickCount();//RELAYSWITCHMARGINE;
                state = REL_TRIAC_CLS_ON;
                //'break' not required
            }
        case REL_TRIAC_CLS_ON:
            gpio_set_level(PUMP_TRIAC_OUT_PIN, 0);
            pump_is_on=1;
            dig_ins.pump_last_start_pure=dig_ins.pump_last_start_2acc=xTaskGetTickCount();
            if ((xTaskGetTickCount()-tout)>RELAYSWITCHMARGINE){
                state = REL_RELAY_CLS_ON;
                tout = xTaskGetTickCount();
            }
            break;
        case REL_RELAY_CLS_ON:
            gpio_set_level(PUMP_RELAY_OUT_PIN, 1);
            if ((xTaskGetTickCount()-tout)>RELAYSWITCHMARGINE){
                state = REL_ON;
                gpio_set_level(PUMP_TRIAC_OUT_PIN, 1);
            }
            break;
        case REL_ON:
            if (relay_onoff_cmd == PP_REL_ON) {
                break;
            } else {
                tout = xTaskGetTickCount();
                state = REL_TRIAC_OPN_ON;
                //'break' not required
            }
        case REL_TRIAC_OPN_ON:
            gpio_set_level(PUMP_TRIAC_OUT_PIN, 0);
            if ((xTaskGetTickCount()-tout)>RELAYSWITCHMARGINE){
                state = REL_RELAY_OPN_OFF;
                tout = xTaskGetTickCount();
            }
            break;
        case REL_RELAY_OPN_OFF:{
            gpio_set_level(PUMP_RELAY_OUT_PIN, 0);
            TickType_t tmp=xTaskGetTickCount();
            if ((tmp-tout)>RELAYSWITCHMARGINE){
                state = REL_OFF;
                gpio_set_level(PUMP_TRIAC_OUT_PIN, 1);
                pump_is_on=0;
                dig_ins.pump_on_acc+=tmp-dig_ins.pump_last_start_2acc;
            }
            tmp=tmp-dig_ins.pump_last_start_pure;//now tmp store pp working time
            if(tmp>dig_ins.time_pp_on_max)dig_ins.time_pp_on_max=tmp;
            if(tmp<dig_ins.time_pp_on_min)dig_ins.time_pp_on_min=tmp;
            break;}
        default:
            tout = xTaskGetTickCount();
            state = REL_TRIAC_OPN_ON;
            break;
    }
}

//**********************************************************************************
#define TEMPR_AMBIENT_DFLT 10.0 //!!!it is default
#define Tmpr_calc_heat_time pdMS_TO_TICKS(20*1000) //!!!it is selected at development!
#define gamma 1.4
static uint16_t tk_gross_vol=24;
static int32_t tk_air_pres=Pressure2ADCcode(1.5);
float SwingRemover(int32_t pres,float v_calc_i);
//*******
void DigInSetTankData(uint16_t vol,int32_t pres){
    tk_gross_vol=vol;
    tk_air_pres=pres;
}
//*******
float TankVolumeCalculation(int32_t pres){
    float v_calc_i;
    static float v_calc_i_p;
    static TickType_t t_p=0;
    float Tmpr_calc_i;
    static float Tmpr_calc_i_p=TEMPR_AMBIENT_DFLT;
    static float p_calc_p=-2;
    TickType_t t_cur;
    double rel_p;
    t_cur=xTaskGetTickCount();
    Tmpr_calc_i=Tmpr_calc_i_p;
    Tmpr_calc_i+=(TEMPR_AMBIENT_DFLT-Tmpr_calc_i_p)*(t_cur-t_p)/(float)Tmpr_calc_heat_time;
    t_p=t_cur;
    if(p_calc_p<0){
        if(p_calc_p<-1){//make shure to have first real adc data
            p_calc_p=-0.5;
            return 24.0;
        }
        t_p=xTaskGetTickCount();
        p_calc_p=pres;
        v_calc_i_p=tk_gross_vol*(float)(tk_air_pres+ADC_1BAR)/(float)(pres+ADC_1BAR);
    }
    p_calc_p=(p_calc_p+ADC_1BAR)*(Tmpr_calc_i+273)/(float)(Tmpr_calc_i_p+273)-ADC_1BAR;
    Tmpr_calc_i_p=Tmpr_calc_i;

    rel_p=(float)(p_calc_p+ADC_1BAR)/(float)(pres+ADC_1BAR);
    v_calc_i=v_calc_i_p*pow(rel_p,(double)(1/gamma));
    Tmpr_calc_i=(Tmpr_calc_i_p+273)*pow(rel_p,(double)(1-gamma)/gamma)-273;       
    p_calc_p=pres;
//    v_calc_i_p=v_calc_i;
    v_calc_i_p=SwingRemover(pres,v_calc_i);
    Tmpr_calc_i_p=Tmpr_calc_i;
    return v_calc_i;
}
//if calculated volume for limited calculation precition will swing to any direction this function
// allow to return it to resanoble values
float SwingRemover(int32_t pres,float v_calc_i){
    static int32_t ref_pres=0;
    static TickType_t tout=0;
    static uint8_t stepped_fg=0;
    float res=v_calc_i;
    if(abs(pres-ref_pres)<Pressure2ADCcode(0.05)){//presure stabilized
        if(!stepped_fg && (xTaskGetTickCount()-tout)>(Tmpr_calc_heat_time*5)){//stabilized for enough time
            float v_indep;
            v_indep=(tk_air_pres+ADC_1BAR)*tk_gross_vol/(float)(pres+ADC_1BAR);
            res=(v_indep-v_calc_i)/10+v_calc_i;//10 - magic number of allowed gradient
            stepped_fg=1;//only 1 time allowed per pressure stabilized
        }         
    }else{
        ref_pres=pres;
        tout=xTaskGetTickCount();
        stepped_fg=0;
    }
    return res;
}