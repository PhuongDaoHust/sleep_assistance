#ifndef __BLE_PROV_H__
#define __BLE_PROV_H__
#include "esp_log.h"
#include <stdbool.h>
#include "esp_event_base.h"
#include "freertos/event_groups.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define GATTC_TAG "WATCH"
#define REMOTE_SERVICE_UUID 0x00FF
#define REMOTE_NOTIFY_CHAR_UUID 0xFF01
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

EventGroupHandle_t ble_event_group;


extern volatile float ACC_intensity_mean;
extern volatile int ACC_count;
extern volatile int HR_count;
extern volatile float HR_mean;


uint8_t device_mac_addr[6];
void get_device_service_name(char *service_name, size_t max);
void ble_start();
void ble_stop_adv();
// void bonded_device_start();
// void __attribute__((unused)) remove_all_bonded_devices(void);


#endif