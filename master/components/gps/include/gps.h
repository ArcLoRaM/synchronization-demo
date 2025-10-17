#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_TAG "GPS"

// DS3231 is not required on master; API removed to avoid old I2C driver usage.

extern volatile uint64_t pps_count;
extern volatile bool time_synchronized;

// Default PPS GPIO (can be overridden at runtime by gps_init_pps_capture)
#define PPS_GPIO_PIN GPIO_NUM_5

// Configurable GPS UART pins (adjust to your Heltec pinout)
// Pick pins that are actually broken out and don't conflict with LoRa (8-14) or OLED I2C (17=SDA,18=SCL)
#ifndef GPS_UART_TX_PIN
#define GPS_UART_TX_PIN GPIO_NUM_2
#endif
#ifndef GPS_UART_RX_PIN
#define GPS_UART_RX_PIN GPIO_NUM_3
#endif

void gps_initialize(void);
void gps_uart_task(void);
void gps_record_pps_pulse(uint64_t ts_us);
// Initialize PPS edge capture using MCPWM on the given GPIO
void gps_init_pps_capture(int pps_gpio);
void synchronize_time_from_gprmc(const char* time_str, const char* date_str);
int64_t time_diff_us(uint64_t time1_us, uint64_t time2_us);
void log_time_comparison_at_pps(const char* filename);
void start_gps_benchmark_tasks(const char* filename);

// ---- Debug helpers ----
uint64_t gps_get_pps_count(void);
uint64_t gps_get_total_captures(void);
uint32_t gps_get_last_capture_age_ms(void);
bool     gps_get_pending_sync(void);
bool     gps_get_time_synchronized(void);
char     gps_get_last_gprmc_status(void);
uint32_t gps_get_last_gprmc_age_ms(void);
void     gps_debug_dump_status(void);

// ---- PPS timing helpers (for PPS-aligned actions) ----
// Returns the esp_timer microseconds timestamp of the last PPS edge.
uint64_t gps_get_last_pps_edge_time_us(void);
// Returns the epoch seconds at the last PPS edge (valid only after sync), or 0 if unknown.
uint64_t gps_get_last_pps_epoch_sec(void);
// Block until the next PPS edge (or timeout). If epoch is known, fills out params.
// Returns true on success, false on timeout.
bool gps_wait_for_next_pps(uint32_t timeout_ms, uint64_t* out_epoch_sec, uint64_t* out_edge_time_us);

// ---- Retained time anchor across deep sleep ----
// Returns true if a valid time anchor (synced epoch) is stored in RTC memory.
bool gps_has_saved_time_anchor(void);
// Clears the saved anchor (forces fresh NMEA sync next boot).
void gps_clear_saved_time_anchor(void);
// Restores time using saved anchor and aligns to next PPS without NMEA.
// On success, sets the system time at the PPS edge and marks time_synchronized=true.
bool gps_restore_time_from_anchor_and_pps(uint32_t pps_timeout_ms);

#ifdef __cplusplus
}
#endif
