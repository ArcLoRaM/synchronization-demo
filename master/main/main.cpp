/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"   // include your HAL implementation
#include <oled_display.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <time.h>
#include <stdlib.h>
#include <ds3231.h>
#include "sdcard.h"
#include "gps.h"

#define RADIO_NSS   (8)   // LoRa CS
#define RADIO_IRQ   (14)  // DIO1
#define RADIO_RST   (12)  // Reset
#define RADIO_GPIO  (13)  // Busy

#define RADIO_SCK   (9)
#define RADIO_MISO  (11)
#define RADIO_MOSI  (10)

#define PPS_GPIO_PIN  GPIO_NUM_4  // GPS PPS input

static const char *TAG = "MASTER";

static DS3231_Info s_ds3231;

extern "C" void IRAM_ATTR pps_isr_handler(void* arg) {
    (void)arg;
    // increment GPS PPS counter
    gps_record_pps_pulse(0);
}

static void init_pps_gpio(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PPS_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ISR service
    static bool isr_installed = false;
    if (!isr_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        isr_installed = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(PPS_GPIO_PIN, pps_isr_handler, nullptr));
}

extern "C" void app_main(void)
{
    // set UTC as timezone
    setenv("TZ", "UTC0", 1);
    tzset();

    ESP_LOGI(TAG, "Initializing SD card...");
    sdcardinitialize();

    ESP_LOGI(TAG, "Initializing DS3231...");
    ds3231_init_info(&s_ds3231, I2C_NUM_0, GPIO_NUM_48, GPIO_NUM_47, 1000);

    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t sqw = ds3231_enable_sqw_1hz(&s_ds3231, true);
    if (sqw == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 SQW 1 Hz enabled");
    } else {
        ESP_LOGW(TAG, "Failed to enable SQW: %d", (int)sqw);
    }

    // make DS3231 available to GPS module for PPS-aligned settimeofday()
    gps_register_ds3231(&s_ds3231);

    ESP_LOGI(TAG, "Initializing GPS + PPS GPIO...");
    init_pps_gpio();
    gps_initialize();

    // start GPS UART task (this is where NMEA parsing + sync arming occurs)
    xTaskCreate((TaskFunction_t)gps_uart_task, "gps_uart_task", 4096, NULL, 5, NULL);

    // wait for at least one PPS and NMEA-based sync
    ESP_LOGI(TAG, "Waiting for PPS and GPS time sync...");
    const int timeout_ms = 5 * 60 * 1000; // 5 minutes
    int waited_ms = 0;
    while (!time_synchronized && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waited_ms += 100;
    }

    if (time_synchronized) {
        ESP_LOGI(TAG, "Synchronized with GPS at PPS; DS3231 set at the same edge.");
    } else {
        ESP_LOGW(TAG, "Timeout waiting for GPS sync.");
    }

    // ==== OLED Initialization ====
    ESP_ERROR_CHECK(oled_init());
    oled_clear();
    oled_draw_text(0, 0, "Heltec ESP32-S3");
    oled_draw_text(0, 1, "Initializing...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    // ==== LoRa Setup ====
    Esp32S3Hal hal(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
    Module mod(&hal, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
    SX1262 radio(&mod);

    int16_t state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Radio initialized OK");
        oled_clear();
        oled_draw_text(0, 0, "LoRa Init OK!");
    } else {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        oled_clear();
        oled_draw_text(0, 0, "LoRa Init FAIL!");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    // Configure parameters (EU868)
    radio.setFrequency(868.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(9);
    radio.setCodingRate(7);
    radio.setOutputPower(14);
    radio.setCRC(true);

    oled_draw_text(0, 2, "Freq:868.0MHz");
    oled_draw_text(0, 3, "SF:9 BW:125kHz");
    vTaskDelay(pdMS_TO_TICKS(1500));

    oled_clear();
    oled_draw_text(0, 0, "LoRa TX Demo");
    oled_draw_text(0, 1, "----------------");

   }
