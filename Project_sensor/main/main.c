#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "MAX30100.h"
#include "MAX30100_main.h"
#include "ADXL345.h"
#include "SimpleKalmanFilter.h"
#include "HTTP_main.h"
#include "i2cmain.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "protocol_examples_common.h"
#include "math.h"
#include "ble_prov.h"

#define portTICK_RATE_MS_MODIFY (uint32_t)10
#define MOVE_INTENSITY_LEVEL	30
#define HTTP_TIMER				10
#define MAX30100_TIMER			1
#define ADXL345_TIMER			1

static const char *TAG1 = "adxl345";
static const char *TAG2 = "max30100";
static const char *TAG3 = "http";

int acc[3];
int8_t ACC_old[3], ACC_new[3];
volatile float ACC_intensity_mean=0;
volatile int ACC_count=0;
volatile int HR_count=0;
volatile float HR_mean=0;

static void ADXL345Task(void *pvParameters) {
	while (1) {
		getAccelerometerData(acc);
		for(uint8_t indx = 0;indx <3 ; indx++){
			ACC_old[indx] = ACC_new[indx];
			ACC_new[indx] = (int8_t) acc[indx];
		}
		//getacc(xyz);
		int ACC_intensity = abs(ACC_new[0]-ACC_old[0]) + abs(ACC_new[1]-ACC_old[1]) +  abs(ACC_new[2]-ACC_old[2]);
		if(ACC_intensity > MOVE_INTENSITY_LEVEL) {
			ACC_intensity_mean = (ACC_intensity_mean*ACC_count+ACC_intensity)/(ACC_count+1);
			ACC_count = ACC_count+1;
		}
		ESP_LOGI(TAG1,"Old: X=%d,Y=%d,Z=%d",ACC_old[0],ACC_old[1],ACC_old[2]);
		ESP_LOGI(TAG1,"New: X=%d,Y=%d,Z=%d",ACC_new[0],ACC_new[1],ACC_new[2]);
		ESP_LOGI(TAG1,"so lan chuyen dong: %d",ACC_count);
		ESP_LOGI(TAG1,"Intensity: %d \t Intensity mean: %f",ACC_intensity,ACC_intensity_mean);
		vTaskDelay(1000*ADXL345_TIMER / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

static void MAX30100_ReadReg_Task(void *pvParmameters){
	while(1)
	{
		MAX30100_update();
	}
	vTaskDelete(NULL);
}
static void MAX30100_CalHR_Task(void *pvParmameters){
	while(1)
	{
		float HR = MAX30100_getRate();
		vTaskDelay(1000*MAX30100_TIMER / portTICK_RATE_MS);
	
		HR_mean = (HR_mean*HR_count + HR)/(HR_count+1);
		HR_count=HR_count+1;
		ESP_LOGI(TAG2," Nhip tim TB: %f \t Nhip tim tuc thoi: %f",HR_mean,HR);
	}
	vTaskDelete(NULL);
}
static void HTTPTask(void *pvParmameters){
	while(1){
		HTTP_send_data(ACC_count, ACC_intensity_mean, HR_mean);
		ESP_LOGI(TAG3,"\n........HTTP SEND SUCCESSFULY...........\n");
		ACC_count=0;
		ACC_intensity_mean=0;
		HR_mean=0;
		HR_count =0;
		vTaskDelay(1000*HTTP_TIMER / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

static void BLETask(void *pvParmameters){
	
	while(1){
	   
	}
	vTaskDelete(NULL);
}

void app_main() {
	ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ESP_ERROR_CHECK(example_connect()); // Connect wifi
	ESP_LOGI("system","system inited");
	configure_task_periodic();
	I2C_master_init();
	// while(!MAX30100_begin());
    MAX30100_init();
	initAcc();
	ble_start();
	
	xTaskCreate(&ADXL345Task,"Adxl345Task",4096,NULL,4,NULL);
	xTaskCreate(&MAX30100_ReadReg_Task,"MAX30100_ReadReg_Task",4096,NULL,3,NULL);
	xTaskCreate(&MAX30100_CalHR_Task,"MAX30100_CalHR_Task",4096,NULL,5,NULL);
	xTaskCreate(&HTTPTask,"HTTPTask",4096,NULL,5,NULL);
	xTaskCreate(&BLETask,"BLETask",4096,NULL,5,NULL);
}


