#ifndef _ADXL345_H_
#define _ADXL345_H_

#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"
#include "i2cmain.h"

#define DEVICE_ADDRESS (0x53)
/* some important register of ADXL345 sensor */
#define POWER_CTL 0x2D //Power-saving features control
#define DATA_FORMAT 0x31 //Data format control
#define BW_RATE 0x2C //Data rate and power mode control
#define DATAX0 0x32 //X-Axis Data 0
#define DATAX1 0x33 //X-Axis Data 1
#define DATAY0 0x34 //Y-Axis Data 0
#define DATAY1 0x35 //Y-Axis Data 1
#define DATAZ0 0x36 //Z-Axis Data 0
#define DATAZ1 0x37 //Z-Axis Data 1

/* function prototype */
void writeTo(uint8_t DEVICE, uint8_t address, uint8_t val);
uint8_t readFrom(uint8_t DEVICE, uint8_t address, uint8_t num, uint8_t buff);
void initAcc(void);
void getAccelerometerData(int *result);
void getacc(double *xyc);
void number_of_movement(unsigned int number,int *xyz, int *old_xyz);
void store_value(int xyz[3], int old_xyz[3]);
#endif