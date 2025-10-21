#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"
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
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <ds3231.h>
#include "sdcard.h"
// #include "driver/gpio.h" // duplicate include removed

#define RADIO_NSS   (8)   // LoRa CS
#define RADIO_IRQ   (14)  // DIO1
#define RADIO_RST   (12)  // Reset
#define RADIO_GPIO  (13)  // Busy

#define RADIO_SCK   (9)
#define RADIO_MISO  (11)
#define RADIO_MOSI  (10)

static const char *TAG = "CHILD";

static DS3231_Info s_ds3231;

// sleep/wake timing
#define SLEEP_MINUTES      10U
#define GUARD_WINDOW_SEC   30U

static void deep_sleep_for_minutes(uint32_t minutes)
{
    uint64_t us = (uint64_t)minutes * 60ULL * 1000000ULL;
    ESP_LOGI(TAG, "Entering deep sleep for %u minute(s)", minutes);
    //oled_clear();
    char line[32];
    snprintf(line, sizeof(line), "Sleep %" PRIu32 "min", minutes);
    //oled_draw_text(0, 0, line);
    //oled_draw_text(0, 2, "Wakes for guard");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
}

// parse message in format: "TS:<epoch_sec>.<usec>" -> returns true on success
static bool parse_timestamp(const char *msg, uint64_t *epoch_sec, uint32_t *usec)
{
    if (!msg || !epoch_sec || !usec) return false;
    const char *p = strstr(msg, "TS:");
    if (!p) return false;
    p += 3; // skip "TS:"

    // find the dot separator
    const char *dot = strchr(p, '.');
    if (!dot) return false;

    // parse seconds (unsigned long long)
    char sec_buf[24] = {0};
    size_t sec_len = (size_t)(dot - p);
    if (sec_len == 0 || sec_len >= sizeof(sec_buf)) return false;
    memcpy(sec_buf, p, sec_len);
    char *endptr = nullptr;
    unsigned long long secs = strtoull(sec_buf, &endptr, 10);
    if (endptr == sec_buf) return false;

    // parse microseconds (zero-padded up to 6 digits)
    const char *u = dot + 1;
    char usec_buf[16] = {0};
    size_t ulen = strcspn(u, " \r\n\0");
    if (ulen == 0 || ulen >= sizeof(usec_buf)) return false;
    memcpy(usec_buf, u, ulen);
    unsigned long micros = strtoul(usec_buf, &endptr, 10);
    if (endptr == usec_buf) return false;
    if (micros > 999999UL) micros = 999999UL; // clamp

    *epoch_sec = (uint64_t)secs;
    *usec = (uint32_t)micros;
    return true;
}

void init_i2c_ds3231(void){
    // initialize i2c bus for ds3231
    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_34,
            .scl_io_num = GPIO_NUM_35,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = { .enable_internal_pullup = false },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    ESP_LOGI(TAG, "Created I2C bus on port 0 (SDA=34, SCL=35)");
   
    ESP_LOGI(TAG, "Initializing DS3231...");

    ds3231_init_info(&s_ds3231, I2C_NUM_0, GPIO_NUM_34, GPIO_NUM_35, 1000);

    vTaskDelay(pdMS_TO_TICKS(300));


    bool ds3231_present = false;
    esp_err_t pr = i2c_master_probe(bus_handle, 0x68, pdMS_TO_TICKS(300));
    if (pr == ESP_OK) {
        ds3231_present = true;
    }


    if (ds3231_present) {
        esp_err_t en32 = ds3231_enable_32khz(&s_ds3231, true);
        if (en32 == ESP_OK) {
            ESP_LOGI(TAG, "DS3231 32kHz output enabled");
        } else {
            ESP_LOGW(TAG, "Failed to enable DS3231 32kHz output: %d", (int)en32);
        }
    } else {
        ESP_LOGW(TAG, "Skipping DS3231 32kHz enable (device not detected)");
    }
}

static esp_err_t set_ds3231_from_epoch(uint64_t epoch_sec, uint32_t usec)
{
    // Round to nearest second using microseconds
    if (usec >= 500000U) {
        epoch_sec += 1ULL;
    }

    time_t t = (time_t)epoch_sec;
    struct tm utc_tm = {};
    // ensure UTC (GMT) conversion
    if (gmtime_r(&t, &utc_tm) == nullptr) {
        return ESP_FAIL;
    }

    // ds3231_set_time expects tm_year since 1900, tm_mon 0..11, etc. gmtime_r provides that.
    return ds3231_set_time(&s_ds3231, &utc_tm);
}

extern "C" void app_main(void)
{
    // set UTC as timezone to avoid local offsets during conversions
    setenv("TZ", "UTC0", 1);
    tzset();

    ESP_LOGI(TAG, "Initializing SD card...");
    sdcardinitialize();

    // ==== OLED Initialization (creates I2C NG bus on port 0) ====
    //ESP_ERROR_CHECK(oled_init());
    //oled_clear();
    //oled_draw_text(0, 0, "Heltec ESP32-S3");
    //oled_draw_text(0, 1, "Initializing...");

    ESP_LOGI(TAG, "Initializing I2C and DS3231...");
    init_i2c_ds3231();

    // Enable DS3231 32kHz output for use as external RTC slow clock (wire 32kHz pin to XTAL_32K_P/GPIO15)


    // ==== LoRa Setup ====
    Esp32S3Hal hal(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
    Module mod(&hal, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
    SX1262 radio(&mod);

    // Retry radio initialization a few times if needed
    int16_t state = 0;
    {
        int tries = 3;
        for (int i = 1; i <= tries; ++i) {
            state = radio.begin();
            if (state == RADIOLIB_ERR_NONE) break;
            ESP_LOGW(TAG, "radio.begin() failed (%d), try %d/%d", (int)state, i, tries);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Radio initialized OK");
        //oled_clear();
        //oled_draw_text(0, 0, "LoRa Init OK!");
    } else {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        //oled_clear();
        //oled_draw_text(0, 0, "LoRa Init FAIL!");
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

    //oled_draw_text(0, 2, "Freq:868.0MHz");
    //oled_draw_text(0, 3, "SF:9 BW:125kHz");
    vTaskDelay(pdMS_TO_TICKS(1500));

    //oled_clear();

    // Decide mode based on wake cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool timer_wakeup = (cause == ESP_SLEEP_WAKEUP_TIMER);
    /*
    if (timer_wakeup) {
        oled_draw_text(0, 0, "Guard window");
        oled_draw_text(0, 1, "listening...");
    } else {
        oled_draw_text(0, 0, "LoRa RX: Wait TS");
        oled_draw_text(0, 1, "----------------");
    }*/

    // ==== Receive + DS3231 update logic ====
    char rxBuf[96] = {0};
    // If we woke from timer, run only for GUARD_WINDOW_SEC, then sleep again.
    uint64_t guard_end_us = esp_timer_get_time() + (uint64_t)GUARD_WINDOW_SEC * 1000000ULL;
    bool guard_ts_received = false;
    while (true) {
        memset(rxBuf, 0, sizeof(rxBuf));
        int16_t rxState = radio.receive((uint8_t*)rxBuf, sizeof(rxBuf) - 1, 1000 /*ms*/);
        vTaskDelay(pdMS_TO_TICKS(10)); // settle buses and feed watchdog 

        if (rxState == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "RX: %s", rxBuf);
            //oled_clear();
            //oled_draw_text(0, 0, "RX: TS frame");
            //oled_draw_text(0, 2, rxBuf);

            uint64_t epoch = 0;
            uint32_t usec = 0;
            if (parse_timestamp(rxBuf, &epoch, &usec)) {
                if (timer_wakeup) {
                    guard_ts_received = true;
                    ESP_LOGI(TAG, "TS Received");
                }
                esp_err_t setRes = set_ds3231_from_epoch(epoch, usec);
                if (setRes == ESP_OK) {
                    ESP_LOGI(TAG, "DS3231 updated to epoch=%llu (usec=%lu)", (unsigned long long)epoch, (unsigned long)usec);
                    struct tm now = {};
                    if (ds3231_get_time(&s_ds3231, &now) == ESP_OK) {
                        char line[64];
                        snprintf(line, sizeof(line), "%04d-%02d-%02d %02d:%02d:%02d",
                                 now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
                                 now.tm_hour, now.tm_min, now.tm_sec);
                        oled_draw_text(0, 4, "RTC set OK");
                        oled_draw_text(0, 5, line);
                        // If this was the initial sync (not timer wake), start the sleep cycle now
                        if (!timer_wakeup) {
                            vTaskDelay(pdMS_TO_TICKS(500));
                            deep_sleep_for_minutes(SLEEP_MINUTES);
                        }
                    } else {
                        oled_draw_text(0, 4, "RTC read fail");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to set DS3231: %d", (int)setRes);
                    //oled_draw_text(0, 4, "RTC set FAIL");
                }
            } else {
                ESP_LOGW(TAG, "Payload not TS format; ignoring");
                //oled_draw_text(0, 4, "Bad payload");
            }
        } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT) {
            ESP_LOGD(TAG, "RX Timeout");
        } else {
            ESP_LOGE(TAG, "RX Error: %d", rxState);
            //oled_draw_text(0, 6, "RX Error");
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // If we're in guard window mode, exit after GUARD_WINDOW_SEC and sleep again
        if (timer_wakeup && esp_timer_get_time() >= guard_end_us) {
            ESP_LOGI(TAG, "Guard window elapsed (%us) - %s", GUARD_WINDOW_SEC, guard_ts_received ? "TS Received" : "No TS");
            ESP_LOGI(TAG, "Sleeping %umin", SLEEP_MINUTES);
            vTaskDelay(pdMS_TO_TICKS(200));
            deep_sleep_for_minutes(SLEEP_MINUTES);
        }
    }
    // 
}
