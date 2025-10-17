/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"   // include your HAL implementation
#include <oled_display.h>
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
#include "esp_sleep.h"
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include "sdcard.h"
#include "gps.h"

#define RADIO_NSS   (8)   // LoRa CS
#define RADIO_IRQ   (14)  // DIO1
#define RADIO_RST   (12)  // Reset
#define RADIO_GPIO  (13)  // Busy

#define RADIO_SCK   (9)
#define RADIO_MISO  (11)
#define RADIO_MOSI  (10)

static const char *TAG = "MASTER";


extern "C" void app_main(void)
{
    // set UTC as timezone
    setenv("TZ", "UTC0", 1);
    tzset();
    ESP_LOGI(TAG, "Initializing GPS + PPS GPIO...");
    gps_init_pps_capture(PPS_GPIO_PIN);
    gps_initialize();

    ESP_LOGI(TAG, "Initializing SD card...");
    sdcardinitialize();


    // start GPS UART task (this is where NMEA parsing + sync arming occurs)
    xTaskCreate((TaskFunction_t)gps_uart_task, "gps_uart_task", 4096, NULL, 5, NULL);

    // wait for at least one PPS and NMEA-based sync
    ESP_LOGI(TAG, "Waiting for PPS and GPS time sync...");
    const int timeout_ms = 5 * 60 * 1000; // 5 minutes
    int waited_ms = 0;
    while (!time_synchronized && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(200));
        waited_ms += 200;
        if ((waited_ms % 1000) == 0) {
            // periodic debug snapshot
            gps_debug_dump_status();
        }
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

    // configure parameters (EU868)
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
    oled_draw_text(0, 1, "Next TX in:");

    // transmit immediately on next PPS, then start the 10 min countdown
    {
        uint64_t pps_epoch = 0, pps_edge_us = 0;
        if (!gps_wait_for_next_pps(1500 /*ms*/, &pps_epoch, &pps_edge_us)) {
            ESP_LOGW(TAG, "Timeout waiting next PPS; using current time");
        }
        struct timeval tv = {};
        gettimeofday(&tv, NULL);
        uint64_t now_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
        uint32_t frac_us = 0;
        uint64_t epoch_sec = tv.tv_sec;
        if (pps_edge_us != 0) {
            frac_us = (uint32_t)((now_us >= pps_edge_us) ? (now_us - pps_edge_us) : 0ULL);
            epoch_sec = (pps_epoch != 0) ? pps_epoch : tv.tv_sec;
            if (frac_us >= 1000000U) {
                epoch_sec += frac_us / 1000000U;
                frac_us %= 1000000U;
            }
        } else {
            frac_us = (uint32_t)tv.tv_usec;
        }
        char msg[64];
        snprintf(msg, sizeof(msg), "TS:%llu.%06lu", (unsigned long long)epoch_sec, (unsigned long)frac_us);
        ESP_LOGI(TAG, "Transmitting timestamp (PPS-aligned): %s", msg);
        int16_t txState = radio.transmit(msg);
        if (txState == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "TX OK");
            oled_draw_text(0, 3, "TX: OK       ");
        } else {
            ESP_LOGE(TAG, "TX failed: %d", txState);
            oled_draw_text(0, 3, "TX: FAIL     ");
        }
    }

    // periodic transmit loop (every 10 mins), countdown with respect to pps
    const int interval_sec = 10 * 60;
    int seconds_left = interval_sec;
    char line_buf[32];
    while (true) {
        uint64_t pps_epoch = 0, pps_edge_us = 0;
        bool got_pps = gps_wait_for_next_pps(2000 /*ms*/, &pps_epoch, &pps_edge_us);
        if (!got_pps) {
            ESP_LOGW(TAG, "No PPS within 2s; falling back to 1s delay");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // decrement once per second
        if (seconds_left > 0) seconds_left--;

        // update OLED countdown mm:ss
        int mm = seconds_left / 60;
        int ss = seconds_left % 60;
        snprintf(line_buf, sizeof(line_buf), "%02d:%02d", mm, ss);
        oled_draw_text(0, 2, line_buf);

        if (seconds_left == 0) {
            // build TimeStamp using the last PPS edge (we're at/just after the PPS)
            struct timeval tv = {};
            gettimeofday(&tv, NULL);
            uint64_t now_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
            uint64_t last_edge_us = (pps_edge_us != 0) ? pps_edge_us : gps_get_last_pps_edge_time_us();
            uint64_t last_epoch   = (pps_epoch != 0) ? pps_epoch : gps_get_last_pps_epoch_sec();
            uint32_t frac_us = 0;
            uint64_t epoch_sec = tv.tv_sec;
            if (last_edge_us != 0) {
                frac_us = (uint32_t)((now_us >= last_edge_us) ? (now_us - last_edge_us) : 0ULL);
                epoch_sec = (last_epoch != 0) ? last_epoch : tv.tv_sec;
                if (frac_us >= 1000000U) {
                    epoch_sec += frac_us / 1000000U;
                    frac_us %= 1000000U;
                }
            } else {
                frac_us = (uint32_t)tv.tv_usec;
            }
            char msg[64];
            snprintf(msg, sizeof(msg), "TS:%llu.%06lu", (unsigned long long)epoch_sec, (unsigned long)frac_us);
            ESP_LOGI(TAG, "Transmitting timestamp (PPS-aligned): %s", msg);
            int16_t txState = radio.transmit(msg);
            if (txState == RADIOLIB_ERR_NONE) {
                ESP_LOGI(TAG, "TX OK");
                oled_draw_text(0, 3, "TX: OK       ");
            } else {
                ESP_LOGE(TAG, "TX failed: %d", txState);
                oled_draw_text(0, 3, "TX: FAIL     ");
            }
            seconds_left = interval_sec; // reset after sending
        }
    }
}
