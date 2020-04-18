/**
 * \file Driver for PCF8574 compartible remote 8-bit I/O expanders for I2C-bus
 * \author Ruslan V. Uss
 */
#ifndef PCF8574_PCF8574_H_
#define PCF8574_PCF8574_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>



#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

/**
 * Device descriptor
 */
typedef struct i2c_dev
{
  i2c_port_t bus;
  uint8_t addr;
} i2c_dev_t;

#ifdef __cplusplus
extern "C"
{
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

#define I2C_MASTER_SCL_IO           		5                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          			4                /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              		I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   		0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   		0                /*!< I2C master do not need buffer */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

esp_err_t i2c_master_init(void);
void check_i2c_error(const char *tag, esp_err_t error);


#define PCF8574A_ADDR                 		0x3F             /*!< slave address for PCF8574A GPIO Expander */
#define PCF8574_ADDR                 		0x20             /*!< slave address for PCF8574 Gpio Expanderr */
/**
 * \brief Read GPIO port value
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574) (0b0111<A2><A1><A0> for PCF8574A)
 * \return 8-bit GPIO port value
 */
esp_err_t i2c_slave_write(i2c_port_t bus, uint8_t slave_addr, uint8_t *data, uint8_t *buff, size_t len);

/**
 * \brief Read GPIO port value
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * \return 8-bit GPIO port value
 */
uint8_t pcf8574_port_read(i2c_dev_t *dev);

/**
 * \brief Continiously read GPIO port values to buffer
 * @param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * @param buf Target buffer
 * @param len Buffer length
 * @return Number of bytes read
 */
size_t pcf8574_port_read_buf(i2c_dev_t *dev, void *buf, size_t len);

/**
 * \brief Write value to GPIO port
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * \param value GPIO port value
 */
void pcf8574_port_write(const i2c_dev_t *dev, uint8_t value);

/**
 * \brief Continiously write GPIO values to GPIO port
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * @param buf Buffer with values
 * @param len Buffer length
 * @return Number of bytes written
 */
esp_err_t pcf8574_port_write_buf(const i2c_dev_t *dev, void *buf, size_t len);

/**
 * \brief Read input value of a GPIO pin
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * \param num pin number (0..7)
 * \return GPIO pin value
 */
bool pcf8574_gpio_read(i2c_dev_t *dev, uint8_t num);

/**
 * \brief Set GPIO pin output
 * Note this is READ - MODIFY - WRITE operation! Please read PCF8574
 * datasheet first.
 * \param addr I2C register address (0b0100<A2><A1><A0> for PCF8574)(0b0111<A2><A1><A0> for PCF8574A)
 * \param num pin number (0..7)
 * \param value true for high level
 */
void pcf8574_gpio_write(i2c_dev_t *dev, uint8_t num, bool value);

#ifdef __cplusplus
}
#endif

#endif /* PCF8574_PCF8574_H_ */
