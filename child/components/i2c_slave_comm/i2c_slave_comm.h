#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the I2C slave on port 1.
 *        Listens on address 0x42, SDA=GPIO8, SCL=GPIO9.
 */
esp_err_t i2c_slave_comm_init(void);

#ifdef __cplusplus
}
#endif
