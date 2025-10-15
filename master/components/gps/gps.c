#include "gps.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/projdefs.h"
#include "esp_timer.h"
#include "sys/time.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <ds3231.h>
#include <unistd.h>
#include <sys/stat.h>

// access RTC slow clock time in microseconds
extern uint64_t esp_rtc_get_time_us(void);

// ------------------ calibration configuration ------------------
#define CALIBRATION_DURATION_PPS 600  // 10 minutes @ 1 PPS
// ----------------------------------------------------------------

// ------------------ constants ------------------
static DS3231_Info* s_rtc = NULL;  // external RTC handle
void gps_register_ds3231(DS3231_Info* rtc) { s_rtc = rtc; }

static volatile uint64_t rtc32k_sync_us = 0;   // esp_rtc_get_time_us() captured at sync PPS
static volatile uint64_t epoch_sync_us  = 0;   // Epoch microseconds at sync
static volatile bool     rtc32k_inited  = false;

static const int uart_buffer_size = 1024;
static const int GPS_UART_TIMEOUT_MS = 200;
static const int GPS_LOG_EVERY_S = 60;  // periodic console GPS info
// --------------------------------------------------------------------------

volatile uint64_t pps_count = 0;
volatile bool time_synchronized = false;
static struct tm synchronized_time;
static uint64_t sync_timestamp_us = 0;
static volatile bool pending_sync = false;              // armed when we parsed a valid GPRMC, waiting for next PPS
static volatile uint64_t pending_sync_target_sec = 0;   // time_t (seconds since epoch) to set at the PPS
static volatile uint64_t pending_sync_armed_pps = 0;    // pps_count value when we armed the sync

// ------------------ Calibration state ------------------
static bool     s_calibration_active = true;   // true until 10 minute window collected
static bool     s_calibration_done   = false;  // latched after factors computed
static uint64_t s_calib_start_pps    = 0;      // PPS count at calibration start (after sync)
static int64_t  s_calib_xtal_offset0 = 0;      // initial XTAL offset at start
static int64_t  s_calib_ds_offset0   = 0;      // initial DS offset (if valid) at start
static double   s_xtal_ppm_correction = 0.0;   // microseconds drift per second (ppm) to subtract
static double   s_ds_ppm_correction   = 0.0;   // same for DS3231 (if SQW valid)
// --------------------------------------------------------


static FILE* log_file = NULL;


// common function to record a PPS pulse timestamp
void gps_record_pps_pulse(uint64_t ts_us) {
    (void)ts_us; 
    pps_count++;
}

// synchronize the time fetched through GPRMC data from NMEA of the GPS module
void synchronize_time_from_gprmc(const char* time_str, const char* date_str) {
    if (!time_synchronized && strlen(time_str) >= 6 && strlen(date_str) >= 6) {
        // parse time HHMMSS
        int hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        int minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        int seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        
        // parse date DDMMYY
        int day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
        int month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
        int year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
        
        // set up synchronized time structure
        synchronized_time.tm_hour = hours;
        synchronized_time.tm_min = minutes;
        synchronized_time.tm_sec = seconds;
        synchronized_time.tm_mday = day;
        synchronized_time.tm_mon = month - 1;  // tm_mon is 0-based
        synchronized_time.tm_year = year - 1900;  // tm_year is years since 1900
        
    time_t gps_time = mktime(&synchronized_time);
    pending_sync_target_sec = (uint64_t)gps_time + 1ULL;
    pending_sync_armed_pps = pps_count;
    pending_sync = true;

    ESP_LOGI(GPS_TAG, "GPS time parsed (UTC): %04d-%02d-%02d %02d:%02d:%02d; arming sync at next PPS to +1s",
         year, month, day, hours, minutes, seconds);
    }
}

// calculate time difference in microseconds
int64_t time_diff_us(uint64_t time1_us, uint64_t time2_us) {
    return (int64_t)time1_us - (int64_t)time2_us;
}

// ------------------ GPRMC parsing helper -----------------------
static bool parse_gprmc_sentence(const char* sentence,
                                 char raw_time[7],
                                 char raw_date[7],
                                 char* status,
                                 char time_str[32],
                                 char date_str[32]) {
    if (!sentence || !raw_time || !raw_date || !status || !time_str || !date_str) return false;
    char tmp[256];
    strncpy(tmp, sentence, sizeof(tmp)-1); tmp[sizeof(tmp)-1] = '\0';
    char* tokens[16] = {0};
    int count = 0; char* tok = strtok(tmp, ",");
    while (tok && count < 16) { tokens[count++] = tok; tok = strtok(NULL, ","); }
    if (count < 9) return false;
    raw_time[0] = '\0';
    if (tokens[1] && strlen(tokens[1]) >= 6) {
        strncpy(raw_time, tokens[1], 6); raw_time[6] = '\0';
        int h = (tokens[1][0]-'0')*10 + (tokens[1][1]-'0');
        int m = (tokens[1][2]-'0')*10 + (tokens[1][3]-'0');
        int s = (tokens[1][4]-'0')*10 + (tokens[1][5]-'0');
        snprintf(time_str, 32, "%02d:%02d:%02d", h,m,s);
    } else time_str[0] = '\0';
    *status = (tokens[2] && tokens[2][0]) ? tokens[2][0] : 'V';
    raw_date[0] = '\0';
    if (tokens[8] && strlen(tokens[8]) >= 6) {
        strncpy(raw_date, tokens[8], 6); raw_date[6] = '\0';
        char day[3] = {tokens[8][0], tokens[8][1], '\0'};
        char mon[3] = {tokens[8][2], tokens[8][3], '\0'};
        char yr[3]  = {tokens[8][4], tokens[8][5], '\0'};
        snprintf(date_str, 32, "%s/%s/20%s", day, mon, yr);
    } else date_str[0] = '\0';
    return true;
}
// --------------------------------------------------------------------------

void log_time_comparison_at_pps(const char* filename) {
    if (!time_synchronized) return;
    extern uint64_t get_last_rtc32k_at_pps(void);
    extern uint64_t get_last_pps_count_captured(void);
    extern int64_t  get_last_pps_sqw_phase_us(void);
    extern uint64_t get_last_sqw_edge_time_us(void);

    uint64_t rtc32k_at_pps = get_last_rtc32k_at_pps();
    uint64_t pps_k         = get_last_pps_count_captured();
    int64_t  phase_us      = get_last_pps_sqw_phase_us();
    bool ds_valid          = (get_last_sqw_edge_time_us() != 0ULL);

    uint64_t elapsed_real_us = pps_k * 1000000ULL;
    uint64_t elapsed_rtc_us  = rtc32k_at_pps - rtc32k_sync_us;
    uint64_t pps_time_us     = epoch_sync_us + elapsed_real_us;

    // ds3231 time fraction reconstruction
    uint64_t ds_time_us = 0ULL;
    if (ds_valid) {
        uint64_t ds_frac_at_pps = (phase_us >= 0) ? (uint64_t)phase_us : (uint64_t)(1000000LL + phase_us);
        if (ds_frac_at_pps >= 1000000ULL) ds_frac_at_pps %= 1000000ULL;
        ds_time_us = (pps_time_us/1000000ULL)*1000000ULL + ds_frac_at_pps;
    }

    int64_t xtal_offset_us = (int64_t)elapsed_real_us - (int64_t)elapsed_rtc_us; 
    int64_t ds_offset_us   = ds_valid ? ((int64_t)pps_time_us - (int64_t)ds_time_us) : 0;

    // ---------------- Calibration handling ----------------
    if (s_calibration_active) {
        if (s_calib_start_pps == 0) {
            s_calib_start_pps = pps_k;
            s_calib_xtal_offset0 = xtal_offset_us;
            if (ds_valid) s_calib_ds_offset0 = ds_offset_us;
            ESP_LOGI(GPS_TAG, "Calibration started (target %d s window). PPS start=%llu", CALIBRATION_DURATION_PPS, (unsigned long long)s_calib_start_pps);
        } else if (pps_k - s_calib_start_pps >= CALIBRATION_DURATION_PPS) {
            uint64_t delta_pps = (pps_k - s_calib_start_pps);
            int64_t xtal_drift_total = xtal_offset_us - s_calib_xtal_offset0;
            s_xtal_ppm_correction = (double)xtal_drift_total / (double)delta_pps;
            if (ds_valid) {
                int64_t ds_drift_total = ds_offset_us - s_calib_ds_offset0;
                s_ds_ppm_correction = (double)ds_drift_total / (double)delta_pps;
            } else {
                s_ds_ppm_correction = 0.0;
            }
            s_calibration_active = false;
            s_calibration_done = true;
            ESP_LOGI(GPS_TAG, "Calibration complete: XTAL drift total=%lld us over %llu s => %.3f ppm; DS drift total=%lld us => %.3f ppm", (long long)xtal_drift_total, (unsigned long long)delta_pps, s_xtal_ppm_correction, (long long)(ds_valid? (ds_offset_us - s_calib_ds_offset0):0), s_ds_ppm_correction);
        }
    }

    // apply compensation
    int64_t xtal_offset_comp = xtal_offset_us;
    int64_t ds_offset_comp   = ds_offset_us;
    if (s_calibration_done) {
        uint64_t compensated_seconds = (pps_k > s_calib_start_pps) ? (pps_k - s_calib_start_pps) : 0ULL;
        int64_t expected_xtal_drift_us = (int64_t)( (double)compensated_seconds * s_xtal_ppm_correction );
        xtal_offset_comp = xtal_offset_us - expected_xtal_drift_us;
        if (ds_valid) {
            int64_t expected_ds_drift_us = (int64_t)( (double)compensated_seconds * s_ds_ppm_correction );
            ds_offset_comp = ds_offset_us - expected_ds_drift_us;
        }
    }
    // -------------------------------------------------------

    // Open file / header
    if (!log_file) {
        log_file = fopen(filename, "a");
        if (log_file) {
            fseek(log_file, 0, SEEK_END);
            if (ftell(log_file) == 0) {
                fprintf(log_file, "PPS_Count,XTAL_Offset_us,DS_Offset_us,XTAL_Drift_us,DS_Drift_us,XTAL_PPM,DS_PPM,Temp_C\n");
                fflush(log_file); int fd_h = fileno(log_file); if (fd_h >= 0) fsync(fd_h);
                ESP_LOGI(GPS_TAG, "SD_LOG header written to %s", filename);
            }
        } else {
            ESP_LOGE(GPS_TAG, "Cannot open %s", filename); return;
        }
    }

    static bool     s_have_prev_logged = false;
    static uint64_t s_prev_pps_k = 0;
    static int64_t  s_prev_xtal_offset = 0;     
    static int64_t  s_prev_ds_offset   = 0;

    uint64_t delta_pps = s_have_prev_logged ? (pps_k - s_prev_pps_k) : 0;
    int64_t xtal_drift_raw = 0;
    int64_t ds_drift_raw   = 0;
    double xtal_ppm_raw = 0.0;
    double ds_ppm_raw   = 0.0;
    if (s_have_prev_logged && delta_pps > 0) {
        xtal_drift_raw = xtal_offset_us - s_prev_xtal_offset;
        xtal_ppm_raw   = (double)xtal_drift_raw / (double)delta_pps;
        if (ds_valid) {
            ds_drift_raw = ds_offset_us - s_prev_ds_offset;
            ds_ppm_raw   = (double)ds_drift_raw / (double)delta_pps;
        }
    }

    int64_t xtal_drift_comp = xtal_drift_raw;
    int64_t ds_drift_comp   = ds_drift_raw;
    double xtal_ppm_comp = xtal_ppm_raw;
    double ds_ppm_comp   = ds_ppm_raw;
    if (s_calibration_done && s_have_prev_logged && delta_pps > 0) {
        int64_t expected_xtal_interval = (int64_t)((double)delta_pps * s_xtal_ppm_correction);
        xtal_drift_comp = xtal_drift_raw - expected_xtal_interval;
        xtal_ppm_comp = (double)xtal_drift_comp / (double)delta_pps;
        if (ds_valid) {
            int64_t expected_ds_interval = (int64_t)((double)delta_pps * s_ds_ppm_correction);
            ds_drift_comp = ds_drift_raw - expected_ds_interval;
            ds_ppm_comp = (double)ds_drift_comp / (double)delta_pps;
        }
    }

    // Temperature (independent of ds_valid for SQW) if RTC present
    char temp_field[16] = "";
    if (s_rtc) {
        float temp_c = 0.0f;
        if (ds3231_get_temperature(s_rtc, &temp_c) == ESP_OK) {
            snprintf(temp_field, sizeof(temp_field), "%.2f", temp_c);
        }
    }

    // Decide which values to print: pre-calibration (raw) or post-calibration (compensated)
    int64_t log_xtal_offset = s_calibration_done ? xtal_offset_comp : xtal_offset_us;
    int64_t log_ds_offset   = s_calibration_done ? ds_offset_comp   : ds_offset_us;
    int64_t log_xtal_drift  = s_calibration_done ? xtal_drift_comp  : xtal_drift_raw;
    int64_t log_ds_drift    = s_calibration_done ? ds_drift_comp    : ds_drift_raw;
    double  log_xtal_ppm    = s_calibration_done ? xtal_ppm_comp    : xtal_ppm_raw;
    double  log_ds_ppm      = s_calibration_done ? ds_ppm_comp      : ds_ppm_raw;

    if (!s_have_prev_logged) {
        if (ds_valid) fprintf(log_file, "%llu,%lld,%lld,,,,,,%s\n", (unsigned long long)pps_k, (long long)log_xtal_offset, (long long)log_ds_offset, temp_field);
        else          fprintf(log_file, "%llu,%lld,,,,,,%s\n", (unsigned long long)pps_k, (long long)log_xtal_offset, temp_field);
    } else {
        if (ds_valid) fprintf(log_file, "%llu,%lld,%lld,%lld,%lld,%.3f,%.3f,%s\n", (unsigned long long)pps_k, (long long)log_xtal_offset, (long long)log_ds_offset, (long long)log_xtal_drift, (long long)log_ds_drift, log_xtal_ppm, log_ds_ppm, temp_field);
        else          fprintf(log_file, "%llu,%lld,,%lld,,%.3f,,%s\n", (unsigned long long)pps_k, (long long)log_xtal_offset, (long long)log_xtal_drift, log_xtal_ppm, temp_field);
    }
    fflush(log_file); int fd = fileno(log_file); if (fd >= 0) fsync(fd);
    ESP_LOGI(GPS_TAG, "%sLOG: PPS=%llu XTAL_off=%lld DS_off=%lld driftX=%lld driftD=%lld ppmX=%.3f ppmD=%.3f (mode=%s) HERE",
             s_calibration_done?"COMP_":"RAW_", (unsigned long long)pps_k,
             (long long)log_xtal_offset, (long long)(ds_valid?log_ds_offset:0),
             (long long)log_xtal_drift, (long long)log_ds_drift,
             log_xtal_ppm, log_ds_ppm,
             s_calibration_done?"compensated":"calibrating/RAW");

    s_prev_pps_k = pps_k;
    s_prev_xtal_offset = xtal_offset_us;
    if (ds_valid) s_prev_ds_offset = ds_offset_us;
    s_have_prev_logged = true;
    fclose(log_file); log_file = NULL;
}

// checksum guard
static inline bool nmea_has_valid_checksum(const char* sentence) {
    if (!sentence || sentence[0] != '$') return false;
    const char* star = strrchr(sentence, '*');
    if (!star || strlen(star) != 3) return false;
    // compute XOR of all chars between '$' and '*'
    uint8_t calc = 0;
    for (const char* p = sentence + 1; p < star; ++p) {
        calc ^= (uint8_t)(*p);
    }
    char provided_str[3] = { star[1], star[2], '\0' };
    uint8_t provided = (uint8_t)strtol(provided_str, NULL, 16);
    return calc == provided;
}

QueueHandle_t uart_queue;
void gps_initialize(void){
    // setup UART buffered IO with event queue

    // install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    //configure uart parameters
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    
    ESP_LOGI(GPS_TAG, "GPS and PPS initialization complete");
}

void gps_uart_task(void) {
    ESP_LOGI(GPS_TAG, "Starting GPS UART task");

    uint8_t* uart_buffer = (uint8_t*) malloc(uart_buffer_size);
    if (uart_buffer == NULL) {
        ESP_LOGE(GPS_TAG, "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
        return;
    }

    char nmea_sentence[256];
    int sentence_pos = 0;
    uint64_t last_gps_log_time = 0;

    while (1) {
    int len = uart_read_bytes(UART_NUM_2, uart_buffer, uart_buffer_size - 1, pdMS_TO_TICKS(GPS_UART_TIMEOUT_MS));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = uart_buffer[i];
                if (c == '$') {
                    sentence_pos = 0;
                    nmea_sentence[sentence_pos++] = c;
                } else if (sentence_pos > 0) {
                    if (c == '\n') {
                        if (sentence_pos > 10) { // minimum valid NMEA length with checksum
                            nmea_sentence[sentence_pos] = '\0';

                            // trim trailing \r
                            if (sentence_pos > 0 && nmea_sentence[sentence_pos - 1] == '\r') {
                                nmea_sentence[sentence_pos - 1] = '\0';
                                sentence_pos--;
                            }
                            // process only GPRMC with valid checksum
                            if (strncmp(nmea_sentence, "$GPRMC", 6) == 0 && nmea_has_valid_checksum(nmea_sentence)) {
                                char raw_time[7], raw_date[7];
                                char status = 'V';
                                char time_str[32] = "";
                                char date_str[32] = "";
                                if (parse_gprmc_sentence(nmea_sentence, raw_time, raw_date, &status, time_str, date_str)) {
                                    if (!time_synchronized && status == 'A' && raw_time[0] && raw_date[0]) {
                                        synchronize_time_from_gprmc(raw_time, raw_date);
                                    }
                                    uint64_t now = esp_timer_get_time();
                                    if (!time_synchronized || (now - last_gps_log_time) >= (uint64_t)GPS_LOG_EVERY_S * 1000000ULL) {
                                        ESP_LOGI(GPS_TAG, "GPS - Time: %s, Date: %s, Status: %c", time_str[0]?time_str:"N/A", date_str[0]?date_str:"N/A", status);
                                        last_gps_log_time = now;
                                    }
                                }
                            }
                        }
                        sentence_pos = 0; 
                    } else if (c != '\r') { 
                        if (sentence_pos < sizeof(nmea_sentence) - 1) {
                            nmea_sentence[sentence_pos++] = c;
                        } else {
                            sentence_pos = 0;
                        }
                    }
                }
            }
        }
    // If we armed a PPS-aligned sync, complete it right after the next PPS occurs (pps_count incremented)
    if (pending_sync && pps_count > pending_sync_armed_pps) {
            struct timeval tv;
            tv.tv_sec = (time_t)pending_sync_target_sec;
            tv.tv_usec = 0;
            settimeofday(&tv, NULL);

            // Update synchronized_time to the set UTC time
            time_t t = (time_t)pending_sync_target_sec;
            struct tm* utc_tm = gmtime(&t);
            if (utc_tm) {
                synchronized_time = *utc_tm;
            }

            // If external RTC is registered, set it to the same UTC at this PPS edge
            if (s_rtc) {
                struct tm set_tm = synchronized_time; // UTC
                esp_err_t sr = ds3231_set_time(s_rtc, &set_tm);
                if (sr == ESP_OK) {
                    // Clear OSF so future boots know time is valid
                    esp_err_t cr = ds3231_clear_oscillator_stop_flag(s_rtc);
                    bool osf_after = true;
                    esp_err_t gr = ds3231_get_oscillator_stop_flag(s_rtc, &osf_after);
                    if (cr == ESP_OK && gr == ESP_OK && !osf_after) {
                        ESP_LOGI(GPS_TAG, "DS3231 set at PPS to %04d-%02d-%02d %02d:%02d:%02d UTC (OSF cleared)",
                                 set_tm.tm_year + 1900, set_tm.tm_mon + 1, set_tm.tm_mday,
                                 set_tm.tm_hour, set_tm.tm_min, set_tm.tm_sec);
                    } else {
                        ESP_LOGW(GPS_TAG, "DS3231 set at PPS ok, but OSF not confirmed clear (cr=%d, gr=%d, osf=%d)", (int)cr, (int)gr, (int)osf_after);
                    }
                } else {
                    ESP_LOGW(GPS_TAG, "Failed to set DS3231 time at PPS sync (err=%d)", (int)sr);
                }
            }

            sync_timestamp_us = esp_timer_get_time();
            // Anchor XTAL 32 kHz slow-clock time to UTC at this PPS boundary
            epoch_sync_us = (uint64_t)pending_sync_target_sec * 1000000ULL;
            rtc32k_sync_us = esp_rtc_get_time_us();
            rtc32k_inited = true;
            pps_count = 0;  // reset after synchronization at PPS edge
            time_synchronized = true;
            pending_sync = false;
            // calibration/jitter accumulators removed
            ESP_LOGI(GPS_TAG, "ESP32 RTC synchronized on PPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
                     synchronized_time.tm_year + 1900, synchronized_time.tm_mon + 1, synchronized_time.tm_mday,
                     synchronized_time.tm_hour, synchronized_time.tm_min, synchronized_time.tm_sec);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(uart_buffer);
    vTaskDelete(NULL);
}

// function to start both GPS and comparison tasks
void start_gps_benchmark_tasks(const char* filename) {
    // start GPS UART task
    xTaskCreate((TaskFunction_t)gps_uart_task, "gps_uart_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(GPS_TAG, "GPS benchmark tasks started");
}
