#include "lora_onboard.hpp"
#include "esp_log.h"
#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"

#define RADIO_SCK     5
#define RADIO_MISO    3
#define RADIO_MOSI    6
#define RADIO_NSS     7
#define RADIO_IRQ     33
#define RADIO_BUSY    34
#define RADIO_RST     8
#define RADIO_GPIO    RADIOLIB_NC

static const char *TAG = "lora_onboard";

static Esp32S3Hal hal(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
static Module mod(&hal, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_BUSY);
static SX1262 radio(&mod);

esp_err_t lora_onboard_init(const lora_config_t *cfg)
{
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LoRa module initialized");

    radio.setFrequency(cfg->frequency);
    radio.setBandwidth(cfg->bandwidth);
    radio.setSpreadingFactor(cfg->spreading_factor);
    radio.setCodingRate(cfg->coding_rate);
    radio.setOutputPower(cfg->tx_power);
    radio.setCRC(cfg->crc);

    return ESP_OK;
}

esp_err_t lora_onboard_send(const char *msg)
{
    int16_t txState = radio.transmit(msg);
    if (txState == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "TX OK: %s", msg);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "TX failed: %d", txState);
        return ESP_FAIL;
    }
}

esp_err_t lora_onboard_receive(char *buf, size_t len, uint32_t timeout_ms)
{
    int16_t rxState = radio.receive((uint8_t*)buf, len, timeout_ms);
    if (rxState == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "RX OK: %s", buf);
        return ESP_OK;
    } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT) {
        ESP_LOGW(TAG, "RX Timeout");
        return ESP_ERR_TIMEOUT;
    } else {
        ESP_LOGE(TAG, "RX Error: %d", rxState);
        return ESP_FAIL;
    }
}
