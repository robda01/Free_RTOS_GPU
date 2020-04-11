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

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_ltc2309_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_master_ltc2309_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_master_ltc2309_init(i2c_port_t i2c_num);
void i2c_task_ltc2309_example(void *arg);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif /* MAIN_ADC_H_ */
