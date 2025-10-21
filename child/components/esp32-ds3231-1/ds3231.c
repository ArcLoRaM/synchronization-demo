/*
 * Stripped-down ESP-IDF driver for DS3231 high precision RTC module
 *
 * Amended from https://github.com/UncleRus/esp-idf-lib/blob/master/components/ds3231,
 * which in turn was ported from esp-open-rtos, and from
 * https://github.com/UncleRus/esp-idf-lib/blob/master/components/i2cdev
 *
 * In broad brush strokes, amendments include:
 * - Removal of alarm, square wave generator, 32kHz oscillator features
 * - Removal of features permitting reading of the internal temperature
 * - Removes support for platforms other than ESP32
 * - Removes thread safety and rests directly on the ESP-IDF I2C library. If
 *   multiple tasks are to exercise a single I2C port, external synchronization
 *   (mutex/semaphore) is required.
 * - Removes some runtime argument checking
 *
 * MIT License
 *
 * Copyright (C) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (C) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * Copyright (C) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (C) 2018 Ruslan V. Uss <https://github.com/UncleRus>
 * Copyright (C) 2020 Michael Volk <michael-volk@hotmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, this permission notice, and the disclaimer below
 * shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ds3231.h"
#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"

static const char *TAG = "DS3231";

#define DS3231_I2C_ADDR 0x68
#define MAX_APB_TIMEOUT 1048575

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

// Using ESP-IDF v5 I2C NG API; the master bus must be created elsewhere.

static esp_err_t i2c_read_reg(const DS3231_Info *ds3231, uint8_t reg, void *in_data, size_t in_size)
{
    if (!ds3231 || !ds3231->dev || !in_data || in_size == 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(ds3231->dev, &reg, 1, in_data, in_size,
                                       pdMS_TO_TICKS(ds3231->timeoutMs));
}

static esp_err_t i2c_write_reg(const DS3231_Info *ds3231, uint8_t reg, const void *data, size_t size)
{
    if (!ds3231 || !ds3231->dev) return ESP_ERR_INVALID_ARG;
    uint8_t buf[1 + 8];
    if (1 + size > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = reg;
    if (data && size) memcpy(&buf[1], data, size);
    return i2c_master_transmit(ds3231->dev, buf, 1 + size, pdMS_TO_TICKS(ds3231->timeoutMs));
}

void ds3231_init_info(DS3231_Info *ds3231, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t timeoutMs)
{
    (void)sda_gpio; (void)scl_gpio; // NG path ignores pins; bus must exist
    ds3231->port = port;
    ds3231->addr = DS3231_I2C_ADDR;
    ds3231->timeoutMs = timeoutMs;
    ds3231->bus = NULL;
    ds3231->dev = NULL;

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t res = i2c_master_get_bus_handle(port, &bus);
    if (res != ESP_OK || !bus) {
        ESP_LOGE(TAG, "I2C NG bus not found on port %d (err=%d). Init the bus first.", port, (int)res);
        return;
    }
    ds3231->bus = bus;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ds3231->addr,
        .scl_speed_hz    = DS3231_I2C_FREQ_HZ,
    };
    res = i2c_master_bus_add_device(ds3231->bus, &dev_cfg, &ds3231->dev);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DS3231 (0x%02X) to bus %d: %d", ds3231->addr, port, (int)res);
    } else {
        ESP_LOGI(TAG, "DS3231 attached at 0x%02X on I2C port %d", ds3231->addr, port);
    }
}

esp_err_t ds3231_set_time(DS3231_Info *ds3231, struct tm *time)
{
    uint8_t data[7];

    /* time/date data */
    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd(time->tm_year - 100);

    return i2c_write_reg(ds3231, DS3231_ADDR_TIME, data, 7);
}

/* Get a byte containing just the requested bits
 * pass the register address to read, a mask to apply to the register and
 * an uint* for the output
 * you can test this value directly as true/false for specific bit mask
 * of use a mask of 0xff to just return the whole register byte
 * returns true to indicate success
 */
static esp_err_t ds3231_get_flag(DS3231_Info *ds3231, uint8_t addr, uint8_t mask, uint8_t *flag)
{
    uint8_t data = 0;

    /* get register */
    esp_err_t res = i2c_read_reg(ds3231, addr, &data, 1);
    if (res != ESP_OK)
        return res;

    /* return only requested flag */
    *flag = (data & mask);
    return ESP_OK;
}

/* Set/clear bits in a byte register, or replace the byte altogether
 * pass the register address to modify, a byte to replace the existing
 * value with or containing the bits to set/clear and one of
 * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
 * returns true to indicate success
 */
static esp_err_t ds3231_set_flag(DS3231_Info *ds3231, uint8_t addr, uint8_t bits, uint8_t mode)
{
    uint8_t data = 0;

    /* get status register */
    esp_err_t res = i2c_read_reg(ds3231, addr, &data, 1);
    if (res != ESP_OK)
        return res;
    /* clear the flag */
    if (mode == DS3231_REPLACE)
        data = bits;
    else if (mode == DS3231_SET)
        data |= bits;
    else
        data &= ~bits;

    return i2c_write_reg(ds3231, addr, &data, 1);
}

esp_err_t ds3231_get_oscillator_stop_flag(DS3231_Info *ds3231, bool *flag)
{
    uint8_t f = 0;
    esp_err_t res = ds3231_get_flag(ds3231, DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, &f);
    if (res != ESP_OK) return res;
    *flag = (f ? true : false);
    return ESP_OK;
}

esp_err_t ds3231_clear_oscillator_stop_flag(DS3231_Info *ds3231)
{
    return ds3231_set_flag(ds3231, DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, DS3231_CLEAR);
}

esp_err_t ds3231_get_time(DS3231_Info *ds3231, struct tm *time)
{
    uint8_t data[7] = {0};

    /* read time */
    esp_err_t res = i2c_read_reg(ds3231, DS3231_ADDR_TIME, data, 7);
    if (res != ESP_OK) return res;

    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG)
    {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    }
    else time->tm_hour = bcd2dec(data[2]); /* 24H */
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    time->tm_year = bcd2dec(data[6]) + 100;
    time->tm_isdst = 0;

    return ESP_OK;
}

esp_err_t ds3231_get_temperature(DS3231_Info *ds3231, float *temperature_c)
{
    if (!temperature_c) return ESP_ERR_INVALID_ARG;
    uint8_t data[2] = {0};
    esp_err_t res = i2c_read_reg(ds3231, DS3231_ADDR_TEMP_MSB, data, 2);
    if (res != ESP_OK) return res;
    int8_t msb = (int8_t)data[0];
    uint8_t lsb = data[1];
    float temp = (float)msb + (float)((lsb >> 6) & 0x03) * 0.25f;
    *temperature_c = temp;
    ESP_LOGD(TAG, "DS3231 temperature raw=0x%02x 0x%02x -> %.2f C", (uint8_t)msb, lsb, temp);
    return ESP_OK;
}

esp_err_t ds3231_enable_sqw_1hz(DS3231_Info *ds3231, bool enable)
{
    // Read current control register
    uint8_t ctrl = 0;
    esp_err_t res = i2c_read_reg(ds3231, DS3231_ADDR_CONTROL, &ctrl, 1);
    if (res != ESP_OK) return res;

    if (enable) {
        // INTCN=0 (enable SQW), RS2=0, RS1=0 -> 1 Hz
        ctrl &= ~(DS3231_CTRL_INTCN | DS3231_CTRL_RS2 | DS3231_CTRL_RS1);
    } else {
        // Disable SQW by setting INTCN=1 (interrupt mode)
        ctrl |= DS3231_CTRL_INTCN;
    }
    return i2c_write_reg(ds3231, DS3231_ADDR_CONTROL, &ctrl, 1);
}

esp_err_t ds3231_enable_32khz(DS3231_Info *ds3231, bool enable)
{
    // EN32kHz bit lives in status register (0x0F)
    uint8_t stat = 0;
    esp_err_t res = i2c_read_reg(ds3231, DS3231_ADDR_STATUS, &stat, 1);
    if (res != ESP_OK) return res;
    if (enable) {
        stat |= DS3231_STAT_EN32KHZ;
    } else {
        stat &= ~DS3231_STAT_EN32KHZ;
    }
    return i2c_write_reg(ds3231, DS3231_ADDR_STATUS, &stat, 1);
}
