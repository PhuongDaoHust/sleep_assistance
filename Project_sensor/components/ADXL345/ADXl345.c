#include "ADXL345.h"

#define NUM_RES_DATA (6)

static const char *TAG="ADXl345";

void writeTo(uint8_t DEVICE, uint8_t address, uint8_t val){

    if(I2C_writeRegister(DEVICE,address,val)!=ESP_FAIL){
        ESP_LOGE(TAG, "I2C write error");
    }
}

uint8_t readFrom(uint8_t DEVICE, uint8_t address, uint8_t num, uint8_t byte){
    if(I2C_readRegister(DEVICE,address,&byte,num)!=ESP_FAIL){
        ESP_LOGE(TAG, "I2C read error");
    }

    return byte;
}

void initAcc(void){
    // I2C_master_init();
    writeTo(DEVICE_ADDRESS,POWER_CTL,1 << 3); 
    writeTo(DEVICE_ADDRESS,DATA_FORMAT,0x0B); // the device is in full resolution mode and  maintain a 4 mg/LSB scale factor. Range : 16g
    writeTo(DEVICE_ADDRESS,BW_RATE,0x09); //DATA RATE=1001 (no low power mode)
}

void getAccelerometerData(int *result){
    uint8_t res_data = DATAX0;
    uint8_t buffer[NUM_RES_DATA];
    for(uint8_t i=0;i<NUM_RES_DATA;i++){
        buffer[i]=readFrom(DEVICE_ADDRESS,res_data,1,buffer[i]);
        res_data++;
    }

    result[0]= (((int) buffer[1]) << 8) | buffer[0];
    result[1]= (((int) buffer[3]) << 8) | buffer[2];
    result[2]= (((int) buffer[5]) << 8) | buffer[4];
}

void getacc(double *xyz){
    double gains[3]= {0.00376390,0.00376009, 0.00349265};
    int data_resgister[3];
    getAccelerometerData(data_resgister);
    for(uint8_t i=0;i<3;i++){
        xyz[i] = data_resgister[i] * gains[i];
    }
}

void number_of_movement(unsigned int number  ,int *xyz ,  int *old_xyz){
    for(uint8_t i=0;i<3;i++){
        if(abs(xyz[i]-old_xyz[i])>20){
            number +=1;
            break;
        }
    }
}

void store_value(int xyz[3], int old_xyz[3])
{
    for(uint8_t indx = 0;indx <3 ; indx++){
        old_xyz[indx] = xyz[indx];
    }
}
