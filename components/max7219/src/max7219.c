/*
 * Driver for MAX7219/MAX7221
 * Serially Interfaced, 8-Digit LED Display Drivers
 *
 * Part of esp-open-rtos
 * Copyright (C) 2017 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */

#include "max7219.h"
#include <driver/spi.h>
#include <driver/gpio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "max7219_priv.h"

#define SPI_BUS 1

static const char *MAX7219_TAG = "max7219";
//#define MAX7219_DEBUG

#ifdef MAX7219_DEBUG
#include <stdio.h>
#define debug(fmt, ...) printf("%s: " fmt "\n", "MAX7219", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

#define ALL_CHIPS 0xff
#define ALL_DIGITS 8

#define REG_DIGIT_0      (1 << 8)
#define REG_DECODE_MODE  (9 << 8)
#define REG_INTENSITY    (10 << 8)
#define REG_SCAN_LIMIT   (11 << 8)
#define REG_SHUTDOWN     (12 << 8)
#define REG_DISPLAY_TEST (15 << 8)

#define VAL_CLEAR_BCD    0x0f
#define VAL_CLEAR_NORMAL 0x00

//static const char *TAG = "max7219";

uint8_t oled_dc_level = 0;


static esp_err_t oled_set_dc(uint8_t dc)
{
    oled_dc_level = dc;
    return ESP_OK;
}



/*
static const spi_settings_t bus_settings = {
    .mode         = SPI_MODE0,
    .freq_divider = SPI_FREQ_DIV_10M,
    .msb          = true,
    .minimal_pins = true,
    .endianness   = SPI_BIG_ENDIAN
};
*/



static esp_err_t  send(const max7219_display_t *disp, uint8_t chip, uint16_t value)
{
	value= ((value<<8)&0xff00)|((value>>8)&0x00ff);

    static uint16_t buf[MAX7219_MAX_CASCADE_SIZE] = { 0 };
    if (chip == ALL_CHIPS)
    {
        for (uint8_t i = 0; i < disp->cascade_size; i++)
            buf[i] = value;
    }
    else buf[chip] = value;


//    ESP_LOGI(MAX7219_TAG,"B1: %0x B2: %0x == 32B1: %0x",buf[0],buf[1], *((uint32_t *)(buf)));

    spi_trans_t trans = {0};
    trans.mosi = ((uint32_t *)(buf));
    trans.bits.mosi = 8*2*disp->cascade_size;
    oled_set_dc(1);
    spi_trans(HSPI_HOST, &trans);
    return ESP_OK;
}

bool max7219_init(max7219_display_t *disp)
{
    if (!disp->cascade_size || disp->cascade_size > MAX7219_MAX_CASCADE_SIZE)
    {
        debug("Invalid cascade size %d", disp->cascade_size);
        return false;
    }

    uint8_t max_digits = disp->cascade_size * ALL_DIGITS;
    if (!disp->digits || disp->digits > max_digits)
    {
        debug("Invalid digits count %d, max %d", disp->cascade_size, max_digits);
        return false;
    }

//    gpio_enable(disp->cs_pin, GPIO_OUTPUT);
//    gpio_write(disp->cs_pin, true);

    // Shutdown all chips
    max7219_set_shutdown_mode(disp, true);
    // Disable test
    send(disp, ALL_CHIPS, REG_DISPLAY_TEST);
    // Set max scan limit
    send(disp, ALL_CHIPS, REG_SCAN_LIMIT | (ALL_DIGITS - 1));
    // Set normal decode mode & clear display
    max7219_set_decode_mode(disp, false);
    // Set minimal brightness
    max7219_set_brightness(disp, 5);
    // Wake up
    max7219_set_shutdown_mode(disp, false);

    return true;
}

void max7219_set_decode_mode(max7219_display_t *disp, bool bcd)
{
    disp->bcd = bcd;
    send(disp, ALL_CHIPS, REG_DECODE_MODE | (bcd ? 0xff : 0));
    max7219_clear(disp);
}

void max7219_set_brightness(const max7219_display_t *disp, uint8_t value)
{
    send(disp, ALL_CHIPS, REG_INTENSITY | (value > MAX7219_MAX_BRIGHTNESS ? MAX7219_MAX_BRIGHTNESS : value));
}

void max7219_set_shutdown_mode(const max7219_display_t *disp, bool shutdown)
{
    send(disp, ALL_CHIPS, REG_SHUTDOWN | !shutdown);
}

bool max7219_set_digit(const max7219_display_t *disp, uint8_t digit, uint8_t val)
{
    if (digit >= disp->digits)
    {
        debug("Invalid digit: %d", digit);
        return false;
    }

    if (disp->mirrored)
        digit = disp->digits - digit - 1;

    uint8_t c = digit / ALL_DIGITS;
    uint8_t d = digit % ALL_DIGITS;

    send(disp, c, (REG_DIGIT_0 + ((uint16_t)d << 8)) | val);

    return true;
}

void max7219_clear(const max7219_display_t *disp)
{
    uint8_t val = disp->bcd ? VAL_CLEAR_BCD : VAL_CLEAR_NORMAL;
    for (uint8_t i = 0; i < ALL_DIGITS; i++)
        send(disp, ALL_CHIPS, (REG_DIGIT_0 + ((uint16_t)i << 8)) | val);
}

inline static uint8_t get_char(const max7219_display_t *disp, char c)
{
    if (disp->bcd)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        switch (c)
        {
            case '-':
                return 0x0a;
            case 'E':
            case 'e':
                return 0x0b;
            case 'H':
            case 'h':
                return 0x0c;
            case 'L':
            case 'l':
                return 0x0d;
            case 'P':
            case 'p':
                return 0x0e;
        }
        return VAL_CLEAR_BCD;
    }

    return font_7seg[(c - 0x20) & 0x7f];
}

void max7219_draw_text(const max7219_display_t *disp, uint8_t pos, const char *s)
{
    while (*s!=NULL && pos < disp->digits)
    {
        uint8_t c = get_char(disp, *s);
        if (*(s + 1) == '.')
        {
            c |= 0x80;
            s++;
        }
        max7219_set_digit(disp, pos, c);
        pos++;
        s++;
    }
}
