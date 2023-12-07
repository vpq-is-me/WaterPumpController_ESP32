#include "loop.h"
#include "board.h"
#include "esp_err.h"
#include "esp_random.h"
#include "display_adc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define TAG "LOOP"
#define PP_CAPACITY_AVRG_NUM 10    //number of '10 liters' pulses used for calculating pump capacity
#define PP_MAX_FREQUINT_RESTART 3  //refer to maximum allowed number of too frequent restart pump before disconnect power and rase alarm
#define LITRES_PER_PULSE 10

static void CalcPumpCapacity(float curr_tk_air_vol);
enum {OFF,ON};
static void PumpAlarmShutdown(uint8_t on1_off0);
static void PumpPressostatOnOff(uint8_t on1_off0);
static void RestoreVals(void);
#define StoreVal(name,val) do{                                          \
            nvs_handle_t mem_handle;                                    \
            if(ESP_OK == nvs_open("pump", NVS_READWRITE, &mem_handle)){ \
                nvs_set_i32(mem_handle, name, (val));                    \
                nvs_commit(mem_handle);                                 \
            }                                                           \
            nvs_close(mem_handle);                                      \
            }while(0)
#define ABSi(x) (x<0 ? -x : x)

/// @brief 
typedef struct {
    uint32_t acc_counter; //consumed water counter, in liters
    float pump_capacity;  //pump capacity in liters per minute
    float pump_capacity_avg;  //pump capacity in liters per minute averaged for last PP_CAPACITY_AVRG_NUM pulses
    uint32_t alarm_fgs;   //bitwise alarm flags
    float last_volume_at_WFpulse;//value of air volume in expansion tank at the moment of flow counter pulse
    int16_t pp_touts[4];//some timers concerning pump working, in form of 0.01 secunds. Order of values are in 'touts_arr_en'

    uint32_t pump_on_arr[PP_CAPACITY_AVRG_NUM];//array of last 'PP_CAPACITY_AVRG_NUM' accumulated working on-time of pump from one to next flow-counter pulse
    float vol_bgn_arr[PP_CAPACITY_AVRG_NUM];//array of instant pressure value at moment of flow-counter pulse.
    uint8_t pump_on_arr_idx;//pump_on_acc_arr[] index
    //for avoid unintendant stop pump due to accidental contact bounce frequent restart has some filtering procedure
    uint8_t freq_start_cnt;//counter of consiquent frequent restart of pump before disconnect pump, every frequent start increase by 2, every any pump star decrement
//    uint16_t pres_p; //water presure at the time of previous flow counter pulse (in ADC output format, integer)
    int32_t pres_max;//real min and max pressure of used pressostat, at this values pump was start and sopped in reality(in ADC output format, integer)
    int32_t pres_min; 
    int32_t pres_max_stop;//min and max of software pressostat, at this values pump will start and sopped(in ADC format, integer)
    int32_t pres_min_start; 
    int32_t tk_air_pres;//water tank air pressure from stock, i.e. without water, !converted to ADC output format, integer!
    uint16_t tk_gross_vol;//water tank total(gross) volume, in liters
    uint8_t shutdown_fg;//shutdown flag

} tPumpData_st;

tPumpData_st pump_data = {
    .acc_counter=123, 
    .pump_capacity=0,
    .pump_capacity_avg=0,
    .alarm_fgs=0,
    .pp_touts[0]=0,//for signaling about first measure
    .pump_on_arr={0},
    .vol_bgn_arr={0},
    .pump_on_arr_idx=0,
    .freq_start_cnt=0,
//    .pres_p=-1,
    .pres_max=0,
    .pres_min=0,
    .pres_max_stop=Pressure2ADCcode(3.2),//stop at 3.5bar
    .pres_min_start=Pressure2ADCcode(1.8),//start at 2bar
    .tk_air_pres=Pressure2ADCcode(1.5),// air pressure 1.5bar from stock
    .tk_gross_vol=24,//tank volume
    .shutdown_fg=0,
    };
static uint32_t septic_state=0;
static TickType_t septic_update_time=0;
//allow some time for not receiving data from septic to diside
#define SEPTIC_SILENCE_TIMEOUT pdMS_TO_TICKS(5*60*1000)

void loop_task(void* arg) {
    uint8_t msg_rrobin = 0;//round robin counter for publishing info
    uint32_t msg_opcode;
    uint8_t* msg_p;
    TickType_t tout,start_tick,current_tick;
    EventBits_t event_bits;
    EventGroupHandle_t main_loop_events;
    int32_t last_press_val=-1;
    main_loop_events = xEventGroupCreate();
    DisplayInit(main_loop_events);
    DigInSetTankData(pump_data.tk_gross_vol,pump_data.tk_air_pres);
    DigInInit(main_loop_events);
    RestoreVals();
    DisplaySetCounter(pump_data.acc_counter);
    start_tick=xTaskGetTickCount();
    tout=pdMS_TO_TICKS(3000);
    while (1) {  
        event_bits = xEventGroupWaitBits(main_loop_events, 0xff, pdTRUE, pdFALSE, tout);
        //*************************************************************
        //*******periodical send message block*************************
        current_tick=xTaskGetTickCount();
        if((current_tick-start_tick)>=tout){//time to transmit (publish) new message
            start_tick=current_tick;
            tout=pdMS_TO_TICKS(3000)+(esp_random() & 0x1f)-0x0f;//randomize timeout to avoid collision?
            int msg_length=4;
            switch (msg_rrobin) {
                case 0:
                    msg_opcode = WATER_PUMP_OP_STATUS_COUNTER;
                    msg_p = (uint8_t*)&(pump_data.acc_counter);
                    break;
                case 1:
                    msg_opcode = WATER_PUMP_OP_STATUS_CAPACITY;
                    msg_p = (uint8_t*)&(pump_data.pump_capacity);
                    break;
                case 2:
                    msg_opcode = WATER_PUMP_OP_STATUS_TK_VOL;
                    msg_p = (uint8_t*)&(pump_data.last_volume_at_WFpulse);
                    break;
                case 3:
                    msg_opcode = WATER_PUMP_OP_STATUS_ALARM;
                    msg_p = (uint8_t*)&(pump_data.alarm_fgs);                    
                    break;
                case 4:
                    msg_opcode = WATER_PUMP_OP_STATUS_PP_TIMEOUTS;
                    msg_length=8;
                    msg_p = (uint8_t*)&(pump_data.pp_touts);
                    break;
                default: //case 5
                    msg_opcode = WATER_PUMP_OP_STATUS_CAP_AVG;
                    msg_p = (uint8_t*)&(pump_data.pump_capacity_avg);
                    break;
            }
            if(++msg_rrobin>5)msg_rrobin=0;
            vendor_publish_message(msg_opcode,msg_p,msg_length);
            if(septic_state){
                int changed=(pump_data.alarm_fgs & WP_ALARM_SEPTIC_REQ_SDWN)?0:1;                
                pump_data.alarm_fgs|=WP_ALARM_SEPTIC_REQ_SDWN;
                if(changed)PumpAlarmShutdown(1);
            }
            else {
                int changed=(pump_data.alarm_fgs & WP_ALARM_SEPTIC_REQ_SDWN)?1:0;
                pump_data.alarm_fgs&=~WP_ALARM_SEPTIC_REQ_SDWN; 
                if(changed)PumpAlarmShutdown(0);
            }
            if((current_tick-septic_update_time)>SEPTIC_SILENCE_TIMEOUT)pump_data.alarm_fgs|=WP_ALARM_SEPTIC_NOT_SEND;
            else pump_data.alarm_fgs&=~WP_ALARM_SEPTIC_NOT_SEND;
        }else{//END if(...>=tout)... If we here means we waked by event, next tout must be recalculated
           tout-=(current_tick-start_tick);
           start_tick=current_tick;
        }
        //*************************************************************
        //******events from hardware inputs block**********************
        if (event_bits & EVENT_NEW_COUNT) {
            uint32_t idx=pump_data.pump_on_arr_idx;
            pump_data.last_volume_at_WFpulse=volume_at_WFpulse;
            pump_data.acc_counter += LITRES_PER_PULSE;
            pump_data.pump_on_arr[idx]=pump_last_run_time_ms;
            //to indicate that this is first measirement and timers can be not truth we use negative value
            int16_t pp_on=pump_last_run_time_ms/10;
            if(pump_data.pp_touts[PP_ON_ACC]==0)pump_data.pp_touts[PP_ON_ACC]=-pp_on;
            else pump_data.pp_touts[PP_ON_ACC]=pp_on;            
            pump_data.pp_touts[PP_RUN_MAX]=time_pp_on_max/10;
            pump_data.pp_touts[PP_RUN_MIN]=time_pp_on_min/10;
            if(!time_from_start_at_WFpulse) pump_data.pp_touts[PP_RUN_AT_PULSE]=0;
            else {
                uint16_t t;
                t=time_from_start_at_WFpulse/10;
                if(!t)t=1;
                pump_data.pp_touts[PP_RUN_AT_PULSE]=t;
            }
            // calculate capacity here
            CalcPumpCapacity(volume_at_WFpulse);
            //prepare next cycle
            if (++ idx >= PP_CAPACITY_AVRG_NUM) idx = 0;
            pump_data.pump_on_arr_idx=idx;
            pump_data.vol_bgn_arr[idx]=volume_at_WFpulse;
            DisplaySetCounter(pump_data.acc_counter);
            StoreVal("counter",(*(int32_t*)&pump_data.acc_counter));
        }
        if(event_bits & EVENT_PUMP_STOP){
            pump_data.pres_max=last_press_val;
        }
        if(event_bits & EVENT_PUMP_START){
            if(pump_data.freq_start_cnt)pump_data.freq_start_cnt--;
            else pump_data.alarm_fgs&=~WP_ALARM_FREQUENT_START;
            pump_data.pres_min=last_press_val;
        }
        if(event_bits & EVENT_PUMP_RUN_LONG){
            pump_data.alarm_fgs|=WP_ALARM_PUMP_LONG_RUN;
            PumpAlarmShutdown(1);
        }
        if(event_bits & EVENT_PUMP_TOO_FREQ){
            pump_data.alarm_fgs|=WP_ALARM_FREQUENT_START;
            pump_data.freq_start_cnt+=2;
            if(pump_data.freq_start_cnt>PP_MAX_FREQUINT_RESTART){
                pump_data.alarm_fgs|=WP_ALARM_FREQUENT_START_HIGH;
                PumpAlarmShutdown(1);
            }
        }
        if(event_bits & EVENT_NEW_PRESSURE_VAL){
            last_press_val=ADCGetResult();
            if(last_press_val<0){
                //!!!! check sensor disconnected
                last_press_val=0;
            }
            if(last_press_val<=pump_data.pres_min_start || last_press_val>=pump_data.pres_max_stop){
                PumpPressostatOnOff( last_press_val<=pump_data.pres_min_start? ON : OFF);
            }
        }
        //*************************************************************
        //********extra non essential warning**************************
        if(IsBypassSwitchOn())pump_data.alarm_fgs|=WP_ALARM_BYPASS_SW_ON;
        else pump_data.alarm_fgs&=~WP_ALARM_BYPASS_SW_ON;
    }  // END while(1)
}


static void CalcPumpCapacity(float curr_tk_air_vol){
    uint32_t idx;
    uint32_t working_time=0;
    float vol_bgn;
    idx=pump_data.pump_on_arr_idx;
    if(pump_data.vol_bgn_arr[idx]==0)return;//this is first entrence, there is no enough data
    //calculate capacitance for last 'LITRES_PER_PULSE' liters (for last pulse)
    working_time=pump_data.pump_on_arr[idx];
    vol_bgn=pump_data.vol_bgn_arr[idx];
    if(working_time>1000){//magic number! 1sec. If working time too small for some reason division by near zerro occure 
        float cap_curr=(LITRES_PER_PULSE+vol_bgn-curr_tk_air_vol)*1000/working_time;//*1000 - because of time in milliseconds
        pump_data.pump_capacity=cap_curr;
    }
    //calculate avarage capacity  
    uint32_t consum;
    working_time=0;  
    for(int i=0;i<PP_CAPACITY_AVRG_NUM;i++){
        if(pump_data.vol_bgn_arr[i]==0)return;//at begining array not fully filled
        working_time+=pump_data.pump_on_arr[i];        
    }
    idx=pump_data.pump_on_arr_idx;
    if(++idx>=PP_CAPACITY_AVRG_NUM)idx=0;//try to find oldest cell
    vol_bgn=pump_data.vol_bgn_arr[idx];
    consum=LITRES_PER_PULSE*PP_CAPACITY_AVRG_NUM;
    float cap_avg=(consum+vol_bgn-curr_tk_air_vol)*1000/working_time;//*1000 - because of time in milliseconds
    if((ABSi(pump_data.pump_capacity_avg-cap_avg))>0.01)//to avoid to often renew 
        StoreVal("cap_avg",(*(int32_t*)&cap_avg));
    pump_data.pump_capacity_avg=cap_avg;
}

static void PumpAlarmShutdown(uint8_t set1_res0){
    if(set1_res0){
        pump_data.shutdown_fg=1;
        PumpRelay(PP_REL_OFF);
    }else if(!(pump_data.alarm_fgs & WP_ALARM_SHUTDOWN_MASK))//only if every strict alarms are off
        pump_data.shutdown_fg=0;
}
static void PumpPressostatOnOff(uint8_t on1_off0){
    if(pump_data.shutdown_fg){
        PumpRelay(PP_REL_OFF);
        return;
    }
    PumpRelay(on1_off0 ? PP_REL_ON :PP_REL_OFF);
}


int32_t WPGetParameter(WP_param_id id){
    int32_t res;
    float tmp;
    switch(id){
        case WP_PARAM_I_COUNTER:
            res=pump_data.acc_counter;
            break;
        case WP_PARAM_I_ALARMS:
            res=pump_data.alarm_fgs;
            break;
        case WP_PARAM_I_PRESS_MAX_STOP:
            tmp=ADCcode2Pressure(pump_data.pres_max_stop);
            res=*(int32_t*)&tmp;
            break;
        case WP_PARAM_I_PRESS_MIN_START:
            tmp=ADCcode2Pressure(pump_data.pres_min_start);
            res=*(int32_t*)&tmp;
            break;
        case WP_PARAM_I_AIR_PRESS:
            tmp=ADCcode2Pressure(pump_data.tk_air_pres);
            res=*(int32_t*)&tmp;
            break;
        case WP_PARAM_I_TK_GROSS_VOL:
            res=pump_data.tk_gross_vol;
            break;
        case WP_PARAM_I_SHUTDOWN:
            res=pump_data.shutdown_fg;
            break;
        default:
            res=-1;
            break;
    }
    return res;
}
int32_t WPSetParameter(WP_param_id id, int32_t val){
    int32_t res;
    res=val;
    switch(id){
        case WP_PARAM_I_COUNTER:
            pump_data.acc_counter=*(uint32_t*)&val;
            res=pump_data.acc_counter;
            StoreVal("counter",(*(int32_t*)&pump_data.acc_counter));
            break;
        case WP_PARAM_I_ALARMS:
            pump_data.alarm_fgs=*(uint32_t*)&val;//actualy only reset have sence
            PumpAlarmShutdown(0);
            res=pump_data.alarm_fgs;
            break;
        case WP_PARAM_I_PRESS_MAX_STOP:
            pump_data.pres_max_stop=Pressure2ADCcode(val);
            StoreVal("max_stop",pump_data.pres_max_stop);
            break;
        case WP_PARAM_I_PRESS_MIN_START:
            pump_data.pres_min_start=Pressure2ADCcode(val);
            StoreVal("min_start",pump_data.pres_min_start);
            break;
        case WP_PARAM_I_AIR_PRESS:
            pump_data.tk_air_pres=Pressure2ADCcode(val);
            StoreVal("air_press",pump_data.tk_air_pres);
            break;
        case WP_PARAM_I_TK_GROSS_VOL:
            pump_data.tk_gross_vol=val;
            StoreVal("tk_gross",pump_data.tk_gross_vol);
            break;
        case WP_PARAM_I_SHUTDOWN:
            pump_data.shutdown_fg=val?1:0;
            break;
        default:
            res=-1;
            break;
    }
    return res;    
}

static void RestoreVals(void){
    esp_err_t err;
    nvs_handle_t mem_handle;
    if(ESP_OK == nvs_open("pump", NVS_READWRITE, &mem_handle)){
        int32_t temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"counter",&temp))pump_data.acc_counter=*(uint32_t*)&temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"max_stop",&temp))pump_data.pres_max_stop=temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"min_start",&temp))pump_data.pres_min_start=temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"air_press",&temp))pump_data.tk_air_pres=temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"tk_gross",&temp))pump_data.tk_gross_vol=temp;
        if(ESP_OK==nvs_get_i32(mem_handle,"cap_avg",&temp))pump_data.pump_capacity_avg=*(float*)&temp;
    }
    nvs_close(mem_handle); 
}
void WPSetSepticState(uint8_t dat){
    septic_state=dat;
    septic_update_time=xTaskGetTickCount();
}
