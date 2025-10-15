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


#define RADIO_NSS   (8)   // LoRa CS
#define RADIO_IRQ   (14)  // DIO1
#define RADIO_RST   (12)  // Reset
#define RADIO_GPIO  (13)  // Busy

#define RADIO_SCK   (9)
#define RADIO_MISO  (11)
#define RADIO_MOSI  (10)



static const char *TAG = "LoRa+OLED";





extern "C" void app_main(void)
{
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

    // ==== Transmit loop ====
    uint32_t counter = 0;
    char msg[64];

    while (true) {
        snprintf(msg, sizeof(msg), "Hello #%lu", (unsigned long)counter++);
        ESP_LOGI(TAG, "Sending: %s", msg);

        int16_t txState = radio.transmit(msg);
        // small delay to avoid SPI/I2C overlap
        vTaskDelay(pdMS_TO_TICKS(80));
        oled_clear();
        oled_draw_text(0, 0, "TX Mode");
        oled_draw_text(0, 1, msg);

        if (txState == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "Packet sent successfully!");
            oled_draw_text(0, 3, "Status: OK");
        } else {
            ESP_LOGE(TAG, "Transmit failed: %d", txState);
            oled_draw_text(0, 3, "Status: FAIL");
        }

        // Switch to RX for a short period to check for replies
        int16_t rxState;
        char rxBuf[64] = {0};
      rxState = radio.receive((uint8_t*)rxBuf, sizeof(rxBuf), 3000);
      vTaskDelay(pdMS_TO_TICKS(80));   // allow Vext and bus to settle

        if (rxState == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "RX: %s", rxBuf);
            char info[32];
            snprintf(info, sizeof(info), "RSSI:%.1fdBm", radio.getRSSI());
            oled_draw_text(0, 5, "RX OK!");
            oled_draw_text(0, 6, rxBuf);
            oled_draw_text(0, 7, info);
        } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT) {
            ESP_LOGI(TAG, "RX Timeout");
            oled_draw_text(0, 5, "RX Timeout");
        } else {
            ESP_LOGE(TAG, "RX Error:%d", rxState);
            oled_draw_text(0, 5, "RX Error");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));  // repeat every 3 s
    }
}
