#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/time.h>

#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "ltc2309.h"
#include "hd44780.h"
#include "max7219.h"



static const char *GPIO_TAG = "gpio";
static const char *SPI_TAG = "spi";
static const char *I2C_TAG = "i2c";
static const char *MAIN_TAG = "main";
static const char *LTC2309_TAG = "ltc2309";
static const char *HD44780_TAG = "hd44780";
static const char *MAX7219_TAG = "max7219";


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

#define GPIO_OUTPUT_IO_0    					16
#define GPIO_OUTPUT_IO_1    					2
#define GPIO_OUTPUT_PIN_SEL  					((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     					0
#define GPIO_INPUT_PIN_SEL 						(1ULL<<GPIO_INPUT_IO_0)
#define SPI_WRITE_BUFFER_MAX_SIZE               2048

#define CS_PIN 15
#define DELAY 2000

static max7219_display_t disp = {
    .cs_pin       = CS_PIN,
    .digits       = 8,
    .cascade_size = 1,
    .mirrored     = true
};

static SemaphoreHandle_t semphor = NULL;

xQueueHandle gpio_evt_queue = NULL;
static SemaphoreHandle_t xSemaphore = NULL;

typedef enum {
    SPI_SEND = 0,
    SPI_RECV
} spi_master_mode_t;

static uint8_t oled_dc_level = 0;



static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}



void i2c_task_hd44780(void *arg)
{
	static const uint8_t char_data[] = {
	    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
		0x1f, 0x1f, 0x0e, 0x04, 0x0e, 0x1f, 0x1f, 0x00,
		0x1f, 0x11, 0x0e, 0x04, 0x0e, 0x1f, 0x1f, 0x00,
		0x1f, 0x11, 0x0a, 0x04, 0x0e, 0x1f, 0x1f, 0x00,
		0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x1f, 0x1f, 0x00,
		0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00
	};
	ESP_LOGI(HD44780_TAG, "Setup\n");
	xSemaphore = xSemaphoreCreateMutex();

    if( xSemaphore != NULL )
    {
    	int count = 0;
    	int x =0;
    	if( xSemaphoreTake( xSemaphore, ( TickType_t ) 0 ) )
    	{
    		hd44780_t lcd = {
    					.i2c_dev.bus = I2C_MASTER_NUM,
    					.i2c_dev.addr = PCF8574A_ADDR,
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
    			for(x=0;x<6;x++)
    			{
    				hd44780_upload_character(&lcd, x, char_data + x*8);
    			}

    			hd44780_gotoxy(&lcd, 0, 0);
    			hd44780_puts(&lcd, "\x08 Hello world!");
    			hd44780_gotoxy(&lcd, 0, 1);
    			hd44780_puts(&lcd, "\x09");


			char time[40];
			char icon[4];
			    while (true)
			    {
			    	count++;
			        hd44780_gotoxy(&lcd, 2, 1);
			        hd44780_gotoxy(&lcd, 0, 1);
			        switch(count%5)
			        {
						case 0:
						{
							hd44780_puts(&lcd, "\x09");
							break;
						}
						case 1:
						{
							hd44780_puts(&lcd, "\x0A ");
							break;
						}
						case 2:
						{
							hd44780_puts(&lcd, "\x0B ");
							break;
						}
						case 3:
						{
							hd44780_puts(&lcd, "\x0C ");
							break;
						}
						case 4:
						{
							hd44780_puts(&lcd, "\x0D ");
							break;
						}
						default:
							break;
			        }
			        hd44780_puts(&lcd, icon);
			        sprintf(time, "Testing !! %i", count);
			        snprintf(time, 18, "Testing !! %u     ", count);
			        time[sizeof(time) - 1] = 0;
			        hd44780_gotoxy(&lcd, 2, 1);
			        hd44780_puts(&lcd, time);
			        vTaskDelay(1000 / portTICK_RATE_MS);
			    }
			i2c_driver_delete(I2C_MASTER_NUM);
			xSemaphoreGive( xSemaphore );
        }
    }

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

esp_err_t i2c_master_init()
{
    ESP_LOGI(I2C_TAG, "init i2c");

    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 0;
    conf.clk_stretch_tick = 15; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}


void gpio_init(void)
{
    ESP_LOGI(GPIO_TAG, "init gpio");
    gpio_config_t io_conf;

    //**SETUP OUTPUTS
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


    //SETUP Inputs
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
    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);
}
void spi_master_init(void)
{


    ESP_LOGI(SPI_TAG, "init gpio");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = MAX7219_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    ESP_LOGI(SPI_TAG, "init hspi");
    spi_config_t spi_config;
    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Cancel hardware cs
    spi_config.interface.cs_en = 0;
    // MISO pin is used for DC
    spi_config.interface.miso_en = 0;
    // CPOL: 1, CPHA: 1
    spi_config.interface.cpol = 1;
    spi_config.interface.cpha = 1;
    // Set SPI to master mode
    // 8266 Only support half-duplex
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    spi_config.clk_div = SPI_10MHz_DIV;
    // Register SPI event callback function
    spi_config.event_cb = spi_event_callback;
    spi_init(HSPI_HOST, &spi_config);



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
				//ESP_LOGI(LTC2309_TAG, "*******************\n");
				for (x=0;x<8;x++)
				{

					ret = i2c_master_ltc2309_read(I2C_MASTER_NUM, channel[x], sensor_data);

					if (ret == ESP_OK) {

						Temp = ((double)(int16_t)((sensor_data[0] << 4) | sensor_data[1]>>4)/770);
						//ESP_LOGI(LTC2309_TAG, "TEMP%i: %f\n",x, (float)Temp);
					}
				}
				vTaskDelay(100 / portTICK_RATE_MS);

			}
			i2c_driver_delete(I2C_MASTER_NUM);
			xSemaphoreGive( xSemaphore );
    	}
	}
}


static void IRAM_ATTR spi_event_callback(int event, void *arg)
{
	switch (event) {
        case SPI_INIT_EVENT: {

        }
        break;

        case SPI_TRANS_START_EVENT: {
            gpio_set_level(MAX7219_DC_GPIO, oled_dc_level);
        }
        break;

        case SPI_TRANS_DONE_EVENT: {

        }
        break;

        case SPI_DEINIT_EVENT: {
        }
        break;
    }
}



void spi_max7219_task(void *arg)
{

    max7219_init(&disp);
    //max7219_set_decode_mode(&disp, true);

    char buf[9];
    while (true)
    {
        max7219_clear(&disp);
        max7219_draw_text(&disp, 0, "7219LEDS");
        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        max7219_clear(&disp);
        sprintf(buf, "%2.4f A", 34.6782);
        max7219_draw_text(&disp, 0, buf);
        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        max7219_clear(&disp);
        sprintf(buf, "%08x", esp_random());
        max7219_draw_text(&disp, 0, buf);
        vTaskDelay(DELAY / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{

    semphor = xSemaphoreCreateBinary();

	gpio_init();
	spi_master_init();
	i2c_master_init();

	max7219_display_t disp;

	disp.bcd = 0;
	disp.cascade_size =1;
	disp.cs_pin = 15;
	disp.digits =2;
	disp.mirrored = 0;

    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    //start i2c task
    xTaskCreate(i2c_task_ltc2309, "i2c_task_example1", 2048, NULL, 10, NULL);
    xTaskCreate(i2c_task_hd44780, "i2c_task_example2", 2048, NULL, 10, NULL);
    xTaskCreate(spi_max7219_task, "spi_max7219_task", 2048, NULL, 3, NULL);



    int cnt = 0;
    while (1) {
    	printf("[%d] Hello world!\n", cnt);
        ESP_LOGI(MAIN_TAG, "cnt: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_RATE_MS);
       // gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);

    }


}


