#include "pcf8574.h"


esp_err_t i2c_slave_write(i2c_port_t bus, uint8_t slave_addr, uint8_t *data, uint8_t *buff, size_t len)
{
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr<<1 | WRITE_BIT, ACK_CHECK_EN);	  		//Write address and get ACK	from Slave
    if(data!=NULL)
    	i2c_master_write_byte(cmd, *data, ACK_CHECK_EN);
    else
    	i2c_master_write(cmd, buff, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
int i2c_slave_read(uint8_t bus, uint8_t slave_addr, const uint8_t *data, uint8_t *buf, uint32_t len)
{
    if (i2c_bus_test(bus))
        return -EBUSY;
    if (data != NULL) {
        i2c_start(bus);
        if (!i2c_write(bus, slave_addr << 1))
            goto error;
        if (!i2c_write(bus, *data))
            goto error;
    }
    i2c_start(bus);
    if (!i2c_write(bus, slave_addr << 1 | 1)) // Slave address + read
        goto error;
    while(len) {
        *buf = i2c_read(bus, len == 1);
        buf++;
        len--;
    }
    if (!i2c_stop(bus))
        goto error;
    i2c_bus[bus].flag = false; // Bus free
    return 0;

error:
    debug("Read Error");
    i2c_stop(bus);
    i2c_bus[bus].flag = false; // Bus free
    return -EIO;
*/

esp_err_t pcf8574_port_write_buf(const i2c_dev_t *dev, void *buf, size_t len)
{
    if (!len || !buf) return 0;
    uint8_t *_buf = (uint8_t *)buf;

    if (i2c_slave_write(dev->bus, dev->addr, NULL, _buf, len))
        return 0;
    return len;
}

void pcf8574_port_write(const i2c_dev_t *dev, uint8_t value)
{
    i2c_slave_write(dev->bus, dev->addr, NULL, &value, 1);
}

bool pcf8574_gpio_read(i2c_dev_t *dev, uint8_t num)
{
    return (bool)((pcf8574_port_read(dev) >> num) & 1);
}

void pcf8574_gpio_write(i2c_dev_t *dev, uint8_t num, bool value)
{
    uint8_t bit = (uint8_t)value << num;
    uint8_t mask = ~(1 << num);

    pcf8574_port_write(dev, (pcf8574_port_read(dev) & mask) | bit);
}




/**
 * @brief i2c master initialization
 */


void check_i2c_error(const char *tag, esp_err_t error)
{
	switch(error)
	{
		case	ESP_OK:
		{
			ESP_LOGI(tag, "Success!!\n");
			break;
		}
		case	ESP_ERR_INVALID_ARG:
		{
			ESP_LOGI(tag, "Parameter error!!\n");
			break;
		}
		case	ESP_FAIL:
		{
			ESP_LOGI(tag, "Sending command error, slave doesn't ACK the transfer.!!\n");
			break;
		}
		case	ESP_ERR_INVALID_STATE:
		{
			ESP_LOGI(tag, "I2C driver not installed or not in master mode.!!\n");
			break;
		}
		case	ESP_ERR_TIMEOUT:
		{
			ESP_LOGI(tag, "Operation timeout because the bus is busy.!!\n");
			break;
		}
		default:
		{
			ESP_LOGI(tag, "Unknown!!\n");
			break;
		}
	}
}
