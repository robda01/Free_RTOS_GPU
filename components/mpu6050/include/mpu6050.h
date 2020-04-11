/*
 * mpu6050.h
 *
 *  Created on: 10/04/2020
 *      Author: Darren
 */

#ifndef MAIN_INCLUDE_MPU6050_H_
#define MAIN_INCLUDE_MPU6050_H_

/**
 * @brief i2c master initialization
 */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_mpu6050_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_master_mpu6050_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_master_mpu6050_init(i2c_port_t i2c_num);
void i2c_task_mpu6050_example(void *arg);



#ifdef __cplusplus
}
#endif
#endif /* MAIN_INCLUDE_MPU6050_H_ */
