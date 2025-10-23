#include "i2c_master_comm.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "i2c_master_comm"

#define I2C_MASTER_PORT     1
#define I2C_MASTER_SDA_IO   48
#define I2C_MASTER_SCL_IO   47
#define I2C_MASTER_FREQ_HZ  100000
#define SLAVE_ADDR          0x42  // Must match slave address

static i2c_master_bus_handle_t s_master_bus = NULL;
static i2c_master_dev_handle_t s_slave_dev = NULL;

esp_err_t i2c_master_comm_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,  // synchronous mode
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_master_bus);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SLAVE_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_master_bus, &dev_cfg, &s_slave_dev);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "I2C master initialized on port %d", I2C_MASTER_PORT);
    return ESP_OK;
}

esp_err_t i2c_master_send_message(const char *msg)
{
    if (!msg || !s_slave_dev) return ESP_ERR_INVALID_ARG;

    size_t len = strlen(msg);
    esp_err_t err = i2c_master_transmit(s_slave_dev, (const uint8_t *)msg, len, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent: %s", msg);
    } else {
        ESP_LOGE(TAG, "Transmission failed: %s", esp_err_to_name(err));
    }
    return err;
}
