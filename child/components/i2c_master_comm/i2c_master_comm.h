#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the I2C master on port 1.
 * 
 * This configures GPIO8 (SDA) and GPIO9 (SCL) at 100 kHz,
 * with internal pull-ups enabled.
 */
esp_err_t i2c_master_comm_init(void);

/**
 * @brief Send a string message to the slave.
 *
 * @param msg Null-terminated string.
 * @return ESP_OK on success.
 */
esp_err_t i2c_master_send_message(const char *msg);

#ifdef __cplusplus
}
#endif
