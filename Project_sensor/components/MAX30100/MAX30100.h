#ifndef MAX30100_H
#define MAX30100_H

#include <stdint.h>

#include "MAX30100_Registers.h"
#include "i2cmain.h"


#define DEFAULT_MODE                MAX30100_MODE_SPO2_HR
#define DEFAULT_SAMPLING_RATE       MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH         MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT     MAX30100_LED_CURR_27_1MA
#define DEFAULT_IR_LED_CURRENT      MAX30100_LED_CURR_50MA
#define EXPECTED_PART_ID            0x11
#define CIRCULAR_BUFFER_SIZE        50

typedef struct
{
    uint16_t bufferIR[CIRCULAR_BUFFER_SIZE];
    uint16_t bufferRED[CIRCULAR_BUFFER_SIZE];
    uint8_t writePointer;
    uint8_t readPointer;
}Circular_Buffer;
Circular_Buffer DataBuffer;

uint8_t  CBUFFER_isEmty(Circular_Buffer Cbuff);
uint8_t  CBUFFER_nearlyFull(Circular_Buffer Cbuff);


bool MAX30100_begin();
void MAX30100_setMode(Mode mode);
void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth);
void MAX30100_setSamplingRate(SamplingRate samplingRate);
void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);
void MAX30100_setHighresModeEnabled(bool enabled);

bool MAX30100_getRawValues(uint16_t *ir, uint16_t *red);

void MAX30100_resetFifo();
void MAX30100_startTemperatureSampling();
bool MAX30100_isTemperatureReady();
float MAX30100_retrieveTemperature();
void MAX30100_shutdown();
void MAX30100_resume();
uint8_t MAX30100_getPartId();


uint8_t MAX30100_readRegister(uint8_t address);
void MAX30100_writeRegister(uint8_t address, uint8_t data);
void MAX30100_burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length);
void MAX30100_readFifoData();

#endif
