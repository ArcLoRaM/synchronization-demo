#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize OLED (powers Vext, resets panel, sets up I2C, sends SSD1306 init).
esp_err_t oled_init(void);

// Clear screen (fills 128x64 with zeros).
esp_err_t oled_clear(void);

// Optional: quick I2C bus scan (logs found addresses).
void oled_i2c_scan(void);

// Vext control (LOW = ON, HIGH = OFF on Heltec WiFi LoRa V3).
void oled_vext_on(void);
void oled_vext_off(void);


/**
 * @brief Draw ASCII text at (x, page)
 *
 * @param x       Horizontal pixel position (0–127)
 * @param page    Page number (0–7), each page is 8 pixels tall
 * @param text    Null-terminated string to draw
 * @return ESP_OK on success
 */
esp_err_t oled_draw_text(uint8_t x, uint8_t page, const char *text);
#ifdef __cplusplus
}
#endif
