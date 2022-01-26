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

#define portTICK_RATE_MS_MODIFY (uint32_t)10

static const char *TAG1 = "adxl345";
static const char *TAG2 = "max30100";
//static const char *TAG3 = "HTTP";
uint8_t byte1;
unsigned int num_of_mov=0;
int acc[3],old_acc[3];
float acc_k[3];
double xyz[3],intensity;
int i=0;

SemaphoreHandle_t xSemaphore;

static void ADXL345Task(void *pvParameters) {
	ESP_LOGI(TAG1,"adx345 task started\n");
	initAcc();
	ESP_LOGI(TAG1,"accelerometer started");
	while (1) {
	//xSemaphoreTake(xSemaphore,portMAX_DELAY);
	I2C_readRegister(DEVICE_ADDRESS,0x00,&byte1,1);
	printf("DEVICE: %d",byte1);
	for(uint8_t indx = 0;indx <3 ; indx++){
        old_acc[indx] = acc[indx];
	}
	getAccelerometerData(acc);
	// acc_k[0]= updateEstimate(1.0*acc[1]);
    // acc_k[1]= updateEstimate(1.0*acc[2]);
    // acc_k[2]= updateEstimate(1.0*acc[3]);
		ESP_LOGI(TAG1,"X=%d,Y=%d,Z=%d\n",(int8_t)old_acc[0],(int8_t)old_acc[1],(int8_t)old_acc[2]);
		getacc(xyz);
		//number_of_movement(num_of_mov,acc,old_acc);
		for(uint8_t j=0;j<3;j++){
        if(abs(acc[j]-old_acc[j])>20){
             num_of_mov++;
            break;
        }	
    }	
		intensity = sqrt(pow(abs(acc[1]-old_acc[1]),2) + pow(abs(acc[2]-old_acc[2]),2) +  pow(abs(acc[3]-old_acc[3]),2));
	
		ESP_LOGI(TAG1,"X=%d,Y=%d,Z=%d\n",(int8_t)acc[0],(int8_t)acc[1],(int8_t)acc[2]);
		//ESP_LOGI(TAG1,"X=%f,Y=%f,Z=%f\n",xyz[0],xy[1],acc[2]);
		ESP_LOGI(TAG1,"x =%.3lf g,Y=%.3lf g,Z=%.3lf g",xyz[1],xyz[2],xyz[3]);

		ESP_LOGI(TAG1,"\n so lan chuyen dong: %d",num_of_mov);
		ESP_LOGI(TAG1,"\n Cuong do chuyen dong: %.3f",intensity);
		vTaskDelay(1000 / portTICK_RATE_MS);
		//xSemaphoreGive(xSemaphore);
	}
	
	vTaskDelete(NULL);
}

static void MAX30100Task(void *pvParmameters){
ESP_LOGI(TAG2,"max30100 task started\n");
I2C_master_init();
ESP_LOGI(TAG2,"heart beat started");
while(1){
	//xSemaphoreTake(xSemaphore,portMAX_DELAY);
	printf("\n Heart beat wasn't updated\n");
	MAX30100_update();

	vTaskDelay(1000/portTICK_RATE_MS);
	//xSemaphoreGive(xSemaphore);
}
vTaskDelete(NULL);
}

//static void 

void app_main() {
	ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //ESP_ERROR_CHECK(example_connect()); // Connect wifi
	ESP_LOGI("system","system inited");

	//xSemaphore = xSemaphoreCreateMutex();
	
	
	
	xTaskCreate(&ADXL345Task,	//pvTaskCode
			"Adxl345Task",//pcName
			4096,//usStackDepth
			NULL,//pvParameters
			4,//uxPriority
			NULL//pxCreatedTask
			);

	xTaskCreate(&MAX30100Task,
			"Max30100Task",
			4096,
			NULL,
			4,
			NULL);
}

//HTTP_send_data(1, 2, 3); //example send data


