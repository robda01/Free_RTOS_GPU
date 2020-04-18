#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "ltc2309.h"
#include "hd44780.h"




static const char *MAIN_TAG = "main";
static const char *LTC2309_TAG = "ltc2309";
static const char *GPIO_TAG = "gpio";



/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO16: output
 * GPIO2: output
 * GPIO0:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO15:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO15 with GPIO4
 * Connect GPIO16 with GPIO5
 * Generate pulses on GPIO15/16, that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_OUTPUT_IO_0    16
#define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     0
#define GPIO_INPUT_IO_1     15
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

xQueueHandle gpio_evt_queue = NULL;
SemaphoreHandle_t xSemaphore = NULL;




void i2c_task_hd44780(void *arg)
{
	static const uint8_t char_data[] = {
	    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
	    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00,
		0x1f, 0x1f, 0x0e, 0x04, 0x0e, 0x1f, 0x1f, 0x00
	};

	xSemaphore = xSemaphoreCreateMutex();

    if( xSemaphore != NULL )
    {
    	if( xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) )
    	{
    		hd44780_t lcd = {
    					.i2c_dev.bus = I2C_MASTER_NUM,
    					.i2c_dev.addr = PCF8574_ADDR,
    					.font = HD44780_FONT_5X8,
    					.lines = 2,
    					.pins = {
    						.rs = 0,
    						.e  = 2,
    						.d4 = 4,
    						.d5 = 5,
    						.d6 = 6,
    						.d7 = 7,
    						.bl = 3
    					},
    					.backlight = true
    				};


    			hd44780_init(&lcd);
    			hd44780_upload_character(&lcd, 0, char_data);
    			hd44780_upload_character(&lcd, 1, char_data + 8);
    			hd44780_upload_character(&lcd, 2, char_data + 16);

    			hd44780_gotoxy(&lcd, 0, 0);
    			hd44780_puts(&lcd, "\x08 Hello world!");
    			hd44780_gotoxy(&lcd, 0, 1);
    			hd44780_puts(&lcd, "\x09 \x0A");


			char time[40];

			    while (true)
			    {
			        hd44780_gotoxy(&lcd, 4, 1);
			        sprintf(time, "testing !!");
			        //snprintf(time, 7, "%u     ", 1000 / portTICK_RATE_MS);
			        time[sizeof(time) - 1] = 0;

			        hd44780_puts(&lcd, time);
			        vTaskDelay(1000 / portTICK_RATE_MS);
			    }
			i2c_driver_delete(I2C_MASTER_NUM);
			xSemaphoreGive( xSemaphore );
        }
    }

}

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(GPIO_TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}


void gpio_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO5/2
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *) GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);

}

void i2c_task_ltc2309(void *arg)
{

	uint8_t sensor_data[2];
    float Temp;
    int ret;
    int x;
	xSemaphore = xSemaphoreCreateMutex();


    if( xSemaphore != NULL )
    {
    	if( xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) )
    	{

			while (1) {

				memset(sensor_data, 0, 2);
				ESP_LOGI(LTC2309_TAG, "*******************\n");
				for (x=0;x<8;x++)
				{

					ret = i2c_master_ltc2309_read(I2C_MASTER_NUM, channel[x], sensor_data);

					if (ret == ESP_OK) {

						Temp = ((double)(int16_t)((sensor_data[0] << 4) | sensor_data[1]>>4)/770);
						ESP_LOGI(LTC2309_TAG, "TEMP%i: %f\n",x, (float)Temp);
					}
				}
				vTaskDelay(100 / portTICK_RATE_MS);

			}
			i2c_driver_delete(I2C_MASTER_NUM);
			xSemaphoreGive( xSemaphore );
    	}
	}
}

void app_main(void)
{


	i2c_master_init();
	gpio_init();

    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    //start i2c task
    xTaskCreate(i2c_task_ltc2309, "i2c_task_example1", 2048, NULL, 10, NULL);
    xTaskCreate(i2c_task_hd44780, "i2c_task_example2", 2048, NULL, 10, NULL);


    int cnt = 0;
    while (1) {
    	printf("[%d] Hello world!\n", cnt);
        ESP_LOGI(MAIN_TAG, "cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
       // gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);

    }


}


