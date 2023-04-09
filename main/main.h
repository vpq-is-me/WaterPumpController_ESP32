#ifndef MAIN_H
#define MAIN_H
#include "stdint.h"
#include "esp_ble_mesh_defs.h"

#define CID_ESP 0x02E5

#define WATER_PUMP_OP_SET_PARAM               ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define WATER_PUMP_OP_GET_PARAM               ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define WATER_PUMP_OP_STATUS_PARAM            ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)

#define WATER_PUMP_OP_STATUS_COUNTER            ESP_BLE_MESH_MODEL_OP_3(0x08, CID_ESP)
#define WATER_PUMP_OP_STATUS_CAPACITY           ESP_BLE_MESH_MODEL_OP_3(0x09, CID_ESP)
#define WATER_PUMP_OP_STATUS_TK_VOL             ESP_BLE_MESH_MODEL_OP_3(0x0A, CID_ESP)
#define WATER_PUMP_OP_STATUS_ALARM              ESP_BLE_MESH_MODEL_OP_3(0x0B, CID_ESP)
#define WATER_PUMP_OP_STATUS_PP_TIMEOUTS         ESP_BLE_MESH_MODEL_OP_3(0x0C, CID_ESP)
#define WATER_PUMP_OP_STATUS_CAP_AVG            ESP_BLE_MESH_MODEL_OP_3(0x0D, CID_ESP)

#define SEPTIC_OP_SET               ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define SEPTIC_OP_GET               ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define SEPTIC_OP_ALARM_STATUS      ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define SEPTIC_OP_PEND_ALARM_STATUS ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)



enum {PP_ON_ACC,PP_RUN_MAX,PP_RUN_MIN,PP_RUN_AT_PULSE}touts_arr_en;

void vendor_publish_message(uint32_t op_code, uint8_t*data, uint16_t length);

typedef struct __attribute__ ((packed)) tSetGetMessage_st{
    uint8_t id;
    int32_t val;
}set_get_message_st;

#endif
