#pragma once

#include "driver/uart.h"
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <ds3231.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_TAG "GPS"

void gps_register_ds3231(DS3231_Info* rtc);

extern volatile uint64_t pps_count;
extern volatile bool time_synchronized;

void gps_initialize(void);
void gps_uart_task(void);
void gps_record_pps_pulse(uint64_t ts_us);
void synchronize_time_from_gprmc(const char* time_str, const char* date_str);
int64_t time_diff_us(uint64_t time1_us, uint64_t time2_us);
void log_time_comparison_at_pps(const char* filename);
void start_gps_benchmark_tasks(const char* filename);

#ifdef __cplusplus
}
#endif
