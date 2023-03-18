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

static void CalcPumpCapacity(int32_t curr_pres);
static void CalcWaterTkNetVolume(int32_t curr_pres);
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

typedef struct {
    uint32_t acc_counter; //consumed water counter, in liters
    float pump_capacity;  //pump capacity in liters per minute
    float pump_capacity_avg;  //pump capacity in liters per minute averaged for last PP_CAPACITY_AVRG_NUM pulses
    float tank_volume;    //water accumulator tank (hydrophore) net (useful) volume. Water stored between pump on and off pressures
    uint32_t alarm_fgs;   //bitwise alarm flags

    uint32_t pump_on_arr[PP_CAPACITY_AVRG_NUM];//array of last 'PP_CAPACITY_AVRG_NUM' accumulated working on-time of pump from one to next flow-counter pulse
    int32_t pres_bgn_arr[PP_CAPACITY_AVRG_NUM];//array of instant pressure value at moment of flow-counter pulse.
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
    .tank_volume=3.05,
    .alarm_fgs=0,
    .pump_on_arr={0},
    .pres_bgn_arr={0},
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
            LED_toggle();
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
                    msg_p = (uint8_t*)&(pump_data.tank_volume);
                    break;
                case 3:
                    msg_opcode = WATER_PUMP_OP_STATUS_CAP_AVG;
                    msg_p = (uint8_t*)&(pump_data.pump_capacity_avg);
                    break;
                default: //case 4
                    msg_opcode = WATER_PUMP_OP_STATUS_ALARM;
                    msg_p = (uint8_t*)&(pump_data.alarm_fgs);
                    break;
            }
            if(++msg_rrobin>4)msg_rrobin=0;
            vendor_publish_message(msg_opcode,msg_p,4);
        }else{//END if(...>=tout)... If we here means we waked by event, next tout must be recalculated
           tout-=(current_tick-start_tick);
           start_tick=current_tick;
        }
        //*************************************************************
        //******events from hardware inputs block**********************
        if (event_bits & EVENT_NEW_COUNT) {
            uint32_t idx=pump_data.pump_on_arr_idx;
            pump_data.acc_counter += LITRES_PER_PULSE;
            pump_data.pump_on_arr[idx]=pump_last_run_time_ms;
            // calculate capacity here
            CalcPumpCapacity(press_val_at_cnt_pulse);
            CalcWaterTkNetVolume(press_val_at_cnt_pulse);
int kkk=idx;
int ttt=0;
for(int j=0;j<PP_CAPACITY_AVRG_NUM;j++)ttt+=pump_data.pump_on_arr[j];
if (++ kkk >= PP_CAPACITY_AVRG_NUM) kkk = 0;
ESP_LOGI("PULSE","%d,%.3f,%.3f,%.3f,%d,%d,",pump_data.acc_counter,pump_data.pump_capacity,pump_data.pump_capacity_avg,pump_data.tank_volume,pump_data.pump_on_arr[idx],ttt);
ESP_LOGI("PULSE","%d,%d,%d,%d,%d,%.3f\r\n\r\n",pump_data.pres_bgn_arr[kkk],press_val_at_cnt_pulse,(int)pump_data.pump_on_arr_idx,pump_data.pres_max,pump_data.pres_min,flow_acc_last);
            //prepare next cycle
            if (++ idx >= PP_CAPACITY_AVRG_NUM) idx = 0;
            pump_data.pump_on_arr_idx=idx;
            pump_data.pres_bgn_arr[idx]=press_val_at_cnt_pulse;
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
            PumpAlarmShutdown(OFF);
            pump_data.alarm_fgs|=WP_ALARM_PUMP_LONG_RUN;
        }
        if(event_bits & EVENT_PUMP_TOO_FREQ){
            pump_data.alarm_fgs|=WP_ALARM_FREQUENT_START;
            pump_data.freq_start_cnt+=2;
            if(pump_data.freq_start_cnt>PP_MAX_FREQUINT_RESTART){
                pump_data.alarm_fgs|=WP_ALARM_FREQUENT_START_HIGH;
                PumpAlarmShutdown(OFF);
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


static void CalcPumpCapacity(int32_t curr_pres){
    uint32_t idx;
    uint32_t working_time=0;
    int32_t pres_bgn_abs;
    int32_t curr_pres_abs=curr_pres+ADC_1BAR;
    idx=pump_data.pump_on_arr_idx;
    if(pump_data.pres_bgn_arr[idx]==0)return;//this is first entrence, there is no enough data
    float water_in_tk;//difference of water, stored in tank at this moment and at the begining of averaging period
    //calculate capacitance for last 'LITRES_PER_PULSE' liters (for last pulse)
    working_time=pump_data.pump_on_arr[idx];
    pres_bgn_abs=pump_data.pres_bgn_arr[idx]+ADC_1BAR;
//    int p_dif=curr_pres_abs-pres_bgn_abs;
//!!!    if(ABSi(p_dif)<((pump_data.pres_max-pump_data.pres_min)/2)){ //otherwise calculation probably give too errorneous result
        if(working_time>1000){//magic number! 1sec. If working time too small for some reason division by near zerro occure 
            water_in_tk=(float)(pump_data.tk_gross_vol*(pump_data.tk_air_pres+ADC_1BAR)*(curr_pres_abs-pres_bgn_abs))/(float)(curr_pres_abs*pres_bgn_abs);
            float cap_curr=((float)LITRES_PER_PULSE+water_in_tk)*1000/working_time;//*1000 - because of time in milliseconds
            pump_data.pump_capacity=cap_curr;
        }
//    }
    //calculate avarage capacity  
    uint32_t consum;
    working_time=0;  
    for(int i=0;i<PP_CAPACITY_AVRG_NUM;i++){
        if(pump_data.pres_bgn_arr[i]==0)return;//at begining array not fully filled
        working_time+=pump_data.pump_on_arr[i];        
    }
    idx=pump_data.pump_on_arr_idx;
    if(++idx>=PP_CAPACITY_AVRG_NUM)idx=0;//try to find oldest cell
    pres_bgn_abs=pump_data.pres_bgn_arr[idx]+ADC_1BAR;
    consum=LITRES_PER_PULSE*PP_CAPACITY_AVRG_NUM;
    water_in_tk=(float)(pump_data.tk_gross_vol*(pump_data.tk_air_pres+ADC_1BAR)*(curr_pres_abs-pres_bgn_abs))/(float)(curr_pres_abs*pres_bgn_abs);
    float cap_avg=((float)consum+water_in_tk)*1000/working_time;//*1000 - because of time in milliseconds
//!!!    if((ABSi(pump_data.pump_capacity_avg-cap_avg))>0.05)//to avoid to often renew 
        StoreVal("cap_avg",(*(int32_t*)&cap_avg));
    pump_data.pump_capacity_avg=cap_avg;
}

static void CalcWaterTkNetVolume(int32_t curr_pres){
    float tk_vol;
    int32_t curr_pres_abs=curr_pres+ADC_1BAR;
    if(pump_data.pump_capacity_avg==0)return;//we first time switch on. not ready for calculation
    int32_t pres_bgn_abs=pump_data.pres_bgn_arr[pump_data.pump_on_arr_idx]+ADC_1BAR;
    int p_dif=curr_pres_abs-pres_bgn_abs;
    if(ABSi(p_dif)<((pump_data.pres_max-pump_data.pres_min)/8))return; //we can't do anything, leave previously calculated value otherwise error will be too high
    tk_vol=((float)pump_data.pres_max-pump_data.pres_min)/((float)(pump_data.pres_max+ADC_1BAR) *(pump_data.pres_min+ADC_1BAR));
    tk_vol*=(float)(curr_pres_abs*pres_bgn_abs)/(float)p_dif;
    tk_vol*=(float)(pump_data.pump_on_arr[pump_data.pump_on_arr_idx]*pump_data.pump_capacity_avg)/1000.0-LITRES_PER_PULSE;
    pump_data.tank_volume=tk_vol;
}
static void PumpAlarmShutdown(uint8_t set1_res0){
    if(set1_res0){
        pump_data.shutdown_fg=1;
        PumpRelay(PP_REL_OFF);;
    }else 
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
            pump_data.alarm_fgs=0;//only reset have sence
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

