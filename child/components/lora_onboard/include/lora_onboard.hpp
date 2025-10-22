#pragma once

#include <stdint.h>
#include "esp_err.h"



typedef struct {
    float frequency;
    float bandwidth;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power;
    bool crc;
} lora_config_t;

/**
 * @brief Initialize the onboard LoRa module
 * 
 * @param cfg Configuration parameters (frequency, SF, etc.)
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
 */
esp_err_t lora_onboard_init(const lora_config_t *cfg);

/**
 * @brief Send a LoRa message
 * 
 * @param msg Null-terminated string to send
 * @return esp_err_t ESP_OK if sent successfully
 */
esp_err_t lora_onboard_send(const char *msg);

/**
 * @brief Receive a LoRa message (blocking)
 * 
 * @param buf Pointer to receive buffer
 * @param len Maximum length to receive
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK if message received, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t lora_onboard_receive(char *buf, size_t len, uint32_t timeout_ms);

