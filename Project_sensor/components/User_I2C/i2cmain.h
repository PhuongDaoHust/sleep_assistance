#ifndef I2CMAIN_H
#define I2CMAIN_H
#include <stdio.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define CONFIG_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SDA 21
#define CONFIG_I2C_MASTER_PORT_NUM 0
#define CONFIG_I2C_MASTER_FREQUENCY 100000

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR 0x57                 /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START 0xff                   /*!< Operation mode */
#define ESP_SLAVE_ADDR 0x38                     /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


esp_err_t I2C_master_init(void);
esp_err_t I2C_readRegister(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t *data_rd,uint8_t size);
esp_err_t I2C_writeRegister(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t data_wr);
#endif