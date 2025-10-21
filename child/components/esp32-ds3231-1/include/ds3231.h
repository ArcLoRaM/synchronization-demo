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

#ifndef DS3231_H
#define DS3231_H

#include <esp_err.h>
#include <driver/i2c_master.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS3231_I2C_FREQ_HZ               100000

#define DS3231_STAT_OSCILLATOR    0x80
// Status register bit to enable 32kHz output on 32kHz pin
#define DS3231_STAT_EN32KHZ       0x08

#define DS3231_ADDR_TIME          0x00
#define DS3231_ADDR_CONTROL       0x0e
#define DS3231_ADDR_STATUS        0x0f
#define DS3231_ADDR_TEMP_MSB      0x11
#define DS3231_ADDR_TEMP_LSB      0x12

#define DS3231_12HOUR_FLAG        0x40
#define DS3231_12HOUR_MASK        0x1f
#define DS3231_PM_FLAG            0x20
#define DS3231_MONTH_MASK         0x1f

// Control register (0x0E) bits used for square-wave configuration
#define DS3231_CTRL_INTCN         0x04  // 1 = Interrupt mode (SQW disabled), 0 = SQW enabled
#define DS3231_CTRL_RS1           0x08  // Rate select
#define DS3231_CTRL_RS2           0x10  // Rate select
// For 1 Hz SQW: RS2=0, RS1=0, INTCN=0

enum {
    DS3231_SET = 0,
    DS3231_CLEAR,
    DS3231_REPLACE
};

typedef struct {
    // Target I2C port the bus is created on (0 or 1)
    i2c_port_t port;
    // 7-bit I2C address (0x68)
    uint8_t addr;
    // Timeout for operations in milliseconds
    uint32_t timeoutMs;
    // NG I2C handles (ESP-IDF v5+)
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} DS3231_Info;

/**
 * @brief Initialize device descriptor
 * @param ds3231 DS3231 device descriptor
 * @param port I2C port - I2C_NUM_0 or I2C_NUM_1
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @param timeoutMS timeout for message transmissions
 */
/**
 * Initialize DS3231 descriptor and attach to an existing I2C NG bus on the given port.
 * Note: This function expects the I2C master bus to have been created already using
 * i2c_new_master_bus(). It will retrieve the bus handle with i2c_master_get_bus_handle
 * and add a DS3231 device to it. sda_gpio/scl_gpio are ignored in NG mode and kept
 * for API compatibility.
 */
void ds3231_init_info(DS3231_Info *ds3231, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t timeoutMs);

/**
 * @brief Set the time on the RTC
 *
 * Timezone agnostic, pass whatever you like.
 * I suggest using GMT and applying timezone and DST when read back.
 *
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_set_time(DS3231_Info *ds3231, struct tm *time);

/**
 * @brief Get the time from the RTC, populates a supplied tm struct
 * @param dev Device descriptor
 * @param[out] time RTC time
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_time(DS3231_Info *ds3231, struct tm *time);

/**
 * @brief Check if oscillator has previously stopped
 *
 * E.g. no power/battery or disabled
 * sets flag to true if there has been a stop
 *
 * @param dev Device descriptor
 * @param[out] flag Stop flag
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_oscillator_stop_flag(DS3231_Info *ds3231, bool *flag);

/**
 * @brief Clear the oscillator stopped flag
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_clear_oscillator_stop_flag(DS3231_Info *ds3231);

/**
 * @brief Enable or disable the 1 Hz square-wave output on the SQW pin
 * When enabled, sets RS2=0, RS1=0 and clears INTCN so SQW outputs 1 Hz.
 */
esp_err_t ds3231_enable_sqw_1hz(DS3231_Info *ds3231, bool enable);

/**
 * @brief Read the internal temperature sensor (in degrees Celsius)
 *
 * The DS3231 updates this register every 64 seconds (or immediately after
 * power-up / time writes). Resolution is 0.25 C. Value is a signed 10-bit
 * two's complement number: MSB at 0x11, and the upper two bits of LSB (0x12)
 * represent the fractional .25 and .5 components.
 *
 * @param ds3231 Device descriptor
 * @param[out] temperature_c Temperature in degrees Celsius
 * @return ESP_OK on success
 */
esp_err_t ds3231_get_temperature(DS3231_Info *ds3231, float *temperature_c);

/**
 * @brief Enable or disable the 32 kHz output on the DS3231 32kHz pin.
 * This sets/clears EN32kHz bit in the status register (0x0F).
 */
esp_err_t ds3231_enable_32khz(DS3231_Info *ds3231, bool enable);

#ifdef __cplusplus
}
#endif

#endif // DS3231_H
