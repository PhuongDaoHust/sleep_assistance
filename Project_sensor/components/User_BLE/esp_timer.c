#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "ble_prov.h"

static void periodic_timer_callback(void* arg);

static void periodic_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);
    printf("Pham Thi Yen Linhh \n");
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};
    if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        
        char write_char_data[20] = {0};
        sprintf(write_char_data, "%f-%d-%f", HR_mean, ACC_count, ACC_intensity_mean);
        esp_ble_gattc_write_char(gattc_if,
                                 gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                 gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                 sizeof(write_char_data),
                                 (uint8_t *)write_char_data,
                                 ESP_GATT_WRITE_TYPE_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
        ESP_LOGI(GATTC_TAG, "send data success");
}

void configure_task_periodic(void){
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
};