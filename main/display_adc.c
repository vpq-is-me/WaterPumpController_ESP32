#include <stdio.h>
#include "ssd1306.h"
#include "display_adc.h"
#include "esp_log.h"
#include "board.h"
#include "ADS1115.h"
#include "loop.h"


#define I2C_MASTER_SCL_IO 26        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

//************** diplay SSD1306 data **********************
static ssd1306_handle_t ssd1306_dev = NULL;
void DispLoop(TimerHandle_t tmr);
static uint32_t press=0;
static uint32_t counter=0;
static EventGroupHandle_t input_events;

void DisplaySetPressure(uint32_t new_press){
    if(new_press>999)press=999;
    else press=new_press;
}
void DisplaySetCounter(uint32_t new_counter){
    counter=new_counter;
}

//************** ADS1115 setup ****************************
// Below uses the default values speficied by the datasheet
ads1115_t ads1115_cfg = {
  .reg_cfg =  ADS1115_CFG_LS_COMP_MODE_TRAD | // Comparator is traditional
              ADS1115_CFG_LS_COMP_LAT_NON |   // Comparator is non-latching
              ADS1115_CFG_LS_COMP_POL_LOW |   // Alert is active low
              ADS1115_CFG_LS_COMP_QUE_DIS |   // Compator is disabled
              ADS1115_CFG_LS_DR_16SPS     |   // No. of samples to take
              ADS1115_CFG_MS_PGA_FSR_2_048V|  //PGA set to full scake input signal
              (ADS1115_CFG_MS_MUX_DIFF_AIN0_AIN1<<8) | //input multiplexer AINp=AIN0, AINn=AIN1
              ADS1115_CFG_MS_MODE_CON,        // Mode is set to single-shot
  .dev_addr = 0x48,
};
//*********************************************************

void DisplayInit(EventGroupHandle_t events) {
    i2c_config_t conf;
    input_events=events;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    ADS1115_set_config(&ads1115_cfg);
    ADS1115_request_diff_AIN0_AIN1();

    TimerHandle_t tmr = xTimerCreate("display_tmr", pdMS_TO_TICKS(100), pdTRUE, NULL, DispLoop);
    xTimerStart(tmr, 0);
}
static uint8_t disp_adc_prescaler=0;//adc checked every 100ms, display refpresh every 500ms
static int32_t adc_result;
/// @brief 
/// @param tmr 
void DispLoop(TimerHandle_t tmr){
    adc_result = ADS1115_get_conversion();
    xEventGroupSetBits(input_events,EVENT_NEW_PRESSURE_VAL);
    if(disp_adc_prescaler++<5)return;    
    disp_adc_prescaler=0;

    ssd1306_clear_screen(ssd1306_dev, 0x00);
    char data_str[64] = {0};
    sprintf(data_str, "counter:%d",counter);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)data_str, 16, 1);

    uint32_t tmp=adc_result<0 ? 999: (ADCcode2Pressure(adc_result)*100);//press;
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)"Press:", 16, 1);
    data_str[0]=tmp/100+'0';
    data_str[1]=tmp%100/10+'0';
    data_str[2]=tmp%10+'0';

    ssd1306_draw_1612char(ssd1306_dev,50,0,data_str[0]);
    ssd1306_draw_1612char(ssd1306_dev,50+12,0,'.');
    ssd1306_draw_1612char(ssd1306_dev,50+7+12*1,0,data_str[1]);
    ssd1306_draw_1612char(ssd1306_dev,50+7+12*2,0,data_str[2]);

    if(IsBypassSwitchOn()){
        ssd1306_draw_char(ssd1306_dev,128-16,0,'B',16,0);
    }
    if(IsPumpRun()){
        ssd1306_draw_char(ssd1306_dev,128-8,0,'R',16,0);
    }
    ssd1306_refresh_gram(ssd1306_dev);
}
int32_t ADCGetResult(void){
    return adc_result;
}