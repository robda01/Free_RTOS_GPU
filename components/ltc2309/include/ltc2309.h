/*
 * ltc2309.h
 *
 *  Created on: 7/04/2020
 *      Author: Darren
 */

#ifndef MAIN_ADC_H_
#define MAIN_ADC_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"



#ifdef __cplusplus
extern "C" {
#endif
/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a MPU6050 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO14 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO14/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */



#define I2C_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           4                /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define LTC2309_SENSOR_ADDR                 0x08             /*!< slave address for MPU6050 sensor */
#define LTC2309_SENSOR_GLOBAL_ADDR  		0x6B             /*!< slave address for MPU6050 sensor */


#define LTC2309_CHANNEL_0_COM				0x00
#define LTC2309_CHANNEL_1_COM 				0x40
#define LTC2309_CHANNEL_2_COM				0x10
#define LTC2309_CHANNEL_3_COM				0x50
#define LTC2309_CHANNEL_4_COM				0x20
#define LTC2309_CHANNEL_5_COM				0x60
#define LTC2309_CHANNEL_6_COM				0x30
#define LTC2309_CHANNEL_7_COM				0x70

#define LTC2309_CMD_START                   0x41             /*!< Command to set measure mode */
#define LTC2309_WHO_AM_I                    0x75             /*!< Command to read WHO_AM_I reg */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

/**
 * Define the mpu6050 register address:
 */
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75  /*!< Command to read WHO_AM_I reg */

static const int channel[8]={	LTC2309_CHANNEL_0_COM,\
								LTC2309_CHANNEL_1_COM,\
								LTC2309_CHANNEL_2_COM,\
								LTC2309_CHANNEL_3_COM,\
								LTC2309_CHANNEL_4_COM,\
								LTC2309_CHANNEL_5_COM,\
								LTC2309_CHANNEL_6_COM,\
								LTC2309_CHANNEL_7_COM,};

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_ltc2309_write(i2c_port_t i2c_num, uint8_t *data);
esp_err_t i2c_master_ltc2309_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data);
esp_err_t i2c_master_ltc2309_init(i2c_port_t i2c_num);
void i2c_task_ltc2309_example(void *arg);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif /* MAIN_ADC_H_ */
