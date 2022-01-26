#include "MAX30100.h"
#include "i2cmain.h"
uint8_t  CBUFFER_isEmty(Circular_Buffer Cbuff){
    if (Cbuff.writePointer > Cbuff.readPointer) return 1;
    if ((Cbuff.writePointer< CIRCULAR_BUFFER_SIZE/2) && (Cbuff.readPointer >CIRCULAR_BUFFER_SIZE/2)) 
        return 1;
    return 0; 
}
uint8_t CBUFFER_nearlyFull(Circular_Buffer Cbuff){
    int count, ex;
    if (Cbuff.writePointer > Cbuff.readPointer) ex=0;
    else ex =  CIRCULAR_BUFFER_SIZE;
    
    count = ex + Cbuff.writePointer - Cbuff.readPointer;
    if      (count > 12)    return 4;   
    else if (count > 8)     return 3;
    else if (count > 4)     return 2;
    else                    return 1;
}
bool MAX30100_begin()
{
    if (MAX30100_getPartId() != EXPECTED_PART_ID) {
        return false;
    }
    printf("START SETTING \n");
    MAX30100_setMode(DEFAULT_MODE);
    MAX30100_setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    MAX30100_setSamplingRate(DEFAULT_SAMPLING_RATE);
    MAX30100_setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    MAX30100_setHighresModeEnabled(true);
    MAX30100_resetFifo();
    return true;
}

void MAX30100_setMode(Mode mode)
{
    MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30100_setSamplingRate(SamplingRate samplingRate)
{
    uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    MAX30100_writeRegister(MAX30100_REG_LED_CONFIGURATION, redLedCurrent << 4 | irLedCurrent);
}

void MAX30100_setHighresModeEnabled(bool enabled)
{
    uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
        MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

bool MAX30100_getRawValues(uint16_t *ir, uint16_t *red)
{
    if (CBUFFER_isEmty(DataBuffer)!=0) {
        *ir  = DataBuffer.bufferIR[DataBuffer.readPointer];
        *red = DataBuffer.bufferRED[DataBuffer.readPointer];
        DataBuffer.readPointer += CBUFFER_nearlyFull(DataBuffer);
        if(DataBuffer.readPointer >= CIRCULAR_BUFFER_SIZE) DataBuffer.readPointer =0;
        return true;
    } else {
        return false;
    }
}

void MAX30100_resetFifo()
{
    MAX30100_writeRegister(MAX30100_REG_FIFO_WRITE_POINTER, 0);
    MAX30100_writeRegister(MAX30100_REG_FIFO_READ_POINTER, 0);
    MAX30100_writeRegister(MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

uint8_t MAX30100_readRegister(uint8_t address)
{
    uint8_t buff;
    I2C_readRegister(MAX30100_I2C_ADDRESS,address, &buff, 1);
    return buff;
}

void MAX30100_writeRegister(uint8_t address, uint8_t data)
{
    I2C_writeRegister(MAX30100_I2C_ADDRESS , address,data);
}

void MAX30100_burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    I2C_readRegister(MAX30100_I2C_ADDRESS, baseAddress, buffer, length);
}

void MAX30100_readFifoData()
{
    uint8_t buffer[MAX30100_FIFO_DEPTH*4];
    uint8_t toRead;
    toRead = (MAX30100_readRegister(MAX30100_REG_FIFO_WRITE_POINTER) - MAX30100_readRegister(MAX30100_REG_FIFO_READ_POINTER)) & (MAX30100_FIFO_DEPTH-1);

    if (toRead>0) {
        MAX30100_burstRead(MAX30100_REG_FIFO_DATA, buffer, 4 * toRead);
        for (uint8_t i=0 ; i < toRead ; ++i) {
            //printf("Sample: %d \t Write: %d \t Read: %d \n",toRead, DataBuffer.writePointer,DataBuffer.readPointer);
            DataBuffer.bufferIR[DataBuffer.writePointer]  = (uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]);
            DataBuffer.bufferRED[DataBuffer.writePointer] = (uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3]);
            
            DataBuffer.writePointer +=1;
            if(DataBuffer.writePointer >=CIRCULAR_BUFFER_SIZE) DataBuffer.writePointer =0;
        }
    }
}

void MAX30100_startTemperatureSampling()
{
    uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_TEMP_EN;

    MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

bool MAX30100_isTemperatureReady()
{
    return !(MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float MAX30100_retrieveTemperature()
{
    int8_t tempInteger = MAX30100_readRegister(MAX30100_REG_TEMPERATURE_DATA_INT);
    float tempFrac = MAX30100_readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}

void MAX30100_shutdown()
{
    uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_SHDN;

    MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30100_resume()
{
    uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30100_MC_SHDN;

    MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

uint8_t MAX30100_getPartId()
{
    return MAX30100_readRegister(0xff);
}
