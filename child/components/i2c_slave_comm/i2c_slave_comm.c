#include "i2c_slave_comm.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "i2c_slave_comm"

#define I2C_SLAVE_PORT          1
#define I2C_SLAVE_SDA_IO        33
#define I2C_SLAVE_SCL_IO       34
#define I2C_SLAVE_ADDR          0x42
#define I2C_SLAVE_BUF_DEPTH     256

typedef struct {
    size_t len;
    uint8_t data[96];                // keep it small; adjust as you like
} rx_msg_t;


typedef struct {
    QueueHandle_t rx_queue;
} i2c_slave_ctx_t;

static i2c_slave_ctx_t s_ctx = { 0 };
static i2c_slave_dev_handle_t s_slave = NULL;



// -------- ISR callback: called when master has finished writing to us --------
static bool on_receive_cb(i2c_slave_dev_handle_t i2c_slave,
                          const i2c_slave_rx_done_event_data_t *evt_data,
                          void *user_ctx)
{
    //In ISR Domain, we can't use ESP_LOG, need to defer to a task
    (void)i2c_slave;
    i2c_slave_ctx_t *ctx = (i2c_slave_ctx_t *)user_ctx;
    BaseType_t xTaskWoken = pdFALSE;

    if (evt_data && evt_data->buffer && evt_data->length > 0) {
        rx_msg_t msg;
        size_t n = evt_data->length > sizeof(msg.data) ? sizeof(msg.data) : evt_data->length;
        memcpy(msg.data, evt_data->buffer, n);
        msg.len = n;
        xQueueSendFromISR(ctx->rx_queue, &msg, &xTaskWoken);
    }

    return (xTaskWoken == pdTRUE);
}
/**
 * @brief Simple task that logs the data received from master.
 */
static void rx_logger_task(void *arg)
{
    rx_msg_t msg;
    while (1) {
        if (xQueueReceive(s_ctx.rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            char tmp[129];
            size_t n = msg.len > 128 ? 128 : msg.len;
            memcpy(tmp, msg.data, n);
            tmp[n] = '\0';
            ESP_LOGI(TAG, "Received %u bytes: %s", (unsigned)n, tmp);
        }
    }
}

// ---------- Initialization ----------
esp_err_t i2c_slave_comm_init(void)
{
        s_ctx.rx_queue = xQueueCreate(8, sizeof(rx_msg_t));
    if (!s_ctx.rx_queue) {
        return ESP_ERR_NO_MEM;
    }
    i2c_slave_config_t cfg = {
        .i2c_port = I2C_SLAVE_PORT,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = I2C_SLAVE_BUF_DEPTH,
        .receive_buf_depth = I2C_SLAVE_BUF_DEPTH,
        .slave_addr = I2C_SLAVE_ADDR,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .intr_priority = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ESP_ERROR_CHECK(i2c_new_slave_device(&cfg, &s_slave));


    i2c_slave_event_callbacks_t cbs = {
        .on_request = NULL,
        .on_receive = on_receive_cb,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(s_slave, &cbs, &s_ctx));
   
    xTaskCreate(rx_logger_task, "rx_logger", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "I2C slave initialized on port %d, addr 0x%02X",
             I2C_SLAVE_PORT, I2C_SLAVE_ADDR);
    return ESP_OK;
}


