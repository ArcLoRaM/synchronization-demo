#include "gps.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/projdefs.h"
#include "esp_timer.h"
#include "sys/time.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>
#include "esp_attr.h"

// access RTC slow clock time in microseconds
extern uint64_t esp_rtc_get_time_us(void);

// ------------------ calibration configuration ------------------
#define CALIBRATION_DURATION_PPS 600  // 10 minutes @ 1 PPS
// ----------------------------------------------------------------
#define CAPTURE_WINDOW_SIZE 50

// ------------------ constants ------------------
// No external RTC (DS3231) on master

static volatile uint64_t rtc32k_sync_us = 0;   // esp_rtc_get_time_us() captured at sync PPS
static volatile uint64_t epoch_sync_us  = 0;   // Epoch microseconds at sync
static volatile bool     rtc32k_inited  = false;

static const int uart_buffer_size = 1024;
static const int GPS_UART_TIMEOUT_MS = 200;
static const int GPS_LOG_EVERY_S = 60;  // periodic console GPS info
// --------------------------------------------------------------------------

static portMUX_TYPE cap_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t pps_count = 0;
volatile bool time_synchronized = false;
static struct tm synchronized_time;
static uint64_t sync_timestamp_us = 0;
static volatile bool pending_sync = false;              // armed when we parsed a valid GPRMC, waiting for next PPS
static volatile uint64_t pending_sync_target_sec = 0;   // time_t (seconds since epoch) to set at the PPS
static volatile uint64_t pending_sync_armed_pps = 0;    // pps_count value when we armed the sync
static volatile char s_last_gprmc_status = 'V';
static volatile uint64_t s_last_gprmc_time_us = 0;

// ------------------ Calibration state ------------------
static bool     s_calibration_active = true;   // true until 10 minute window collected
static bool     s_calibration_done   = false;  // latched after factors computed
static uint64_t s_calib_start_pps    = 0;      // PPS count at calibration start (after sync)
static int64_t  s_calib_xtal_offset0 = 0;      // initial XTAL offset at start
static int64_t  s_calib_ds_offset0   = 0;      // initial DS offset (if valid) at start
static double   s_xtal_ppm_correction = 0.0;   // microseconds drift per second (ppm) to subtract
static double   s_ds_ppm_correction   = 0.0;   // same for DS3231 (if SQW valid)
// --------------------------------------------------------

typedef struct {
    uint32_t cap_value;
    uint64_t sys_time_us;
} pps_hw_capture;

static volatile pps_hw_capture captures[CAPTURE_WINDOW_SIZE];
static volatile int capture_index = 0;
static volatile uint64_t total_captures = 0;
static volatile uint64_t last_callback_time = 0;  // last ISR time (us)
static volatile uint64_t last_pps_edge_time_us = 0; // esp_timer time at PPS edge
static volatile uint64_t last_pps_epoch_sec = 0;    // epoch seconds at PPS edge (valid after sync)

// Retained across deep sleep (RTC slow memory)
typedef struct {
    uint64_t epoch_sec_at_sync;   // epoch seconds at last sync PPS
    uint64_t rtc32k_time_at_sync; // esp_rtc_get_time_us() at that PPS
    uint32_t crc32;               // simple CRC to validate
} time_anchor_t;
static RTC_DATA_ATTR time_anchor_t s_time_anchor = {0,0,0};

static uint32_t crc32_simple(const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data; uint32_t c = 0xFFFFFFFFu;
    for (size_t i=0;i<len;i++) { c ^= p[i]; for (int b=0;b<8;b++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1); }
    return c ^ 0xFFFFFFFFu;
}


static FILE* log_file = NULL;

extern volatile uint64_t pps_count;

// hardware capture handles
static mcpwm_cap_timer_handle_t cap_timer = NULL;
static mcpwm_cap_channel_handle_t cap_channel = NULL;
static int s_pps_gpio = -1;  // PPS GPIO selected at runtime

static bool IRAM_ATTR pps_hw_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    // hardware captured timestamp
    uint32_t hw_timestamp = edata->cap_value;

    // system timestamp for comparison
    uint64_t sys_timestamp = esp_timer_get_time();

    // store in circular buffer
    portENTER_CRITICAL_ISR(&cap_mux);
    int idx = capture_index % CAPTURE_WINDOW_SIZE;
    captures[idx].cap_value = hw_timestamp;
    captures[idx].sys_time_us = sys_timestamp;
    capture_index++;
    total_captures++;
    last_callback_time = sys_timestamp;
    last_pps_edge_time_us = sys_timestamp;
    if (time_synchronized && last_pps_epoch_sec != 0) {
        // advance epoch seconds on each PPS edge after initial sync
        last_pps_epoch_sec++;
    }
    portEXIT_CRITICAL_ISR(&cap_mux);

    // feed GPS jitter/PPS accounting with this hardware-captured pulse
    gps_record_pps_pulse(sys_timestamp);

    return false;
}


// common function to record a PPS pulse timestamp
void gps_record_pps_pulse(uint64_t ts_us) {
    (void)ts_us; 
    pps_count++;
}

// ------------------initialize MCPWM hardware capture ------------------------
esp_err_t init_hardware_pps_capture(void) {
    if (s_pps_gpio < 0) {
        // default to header-defined PPS pin if not set
        s_pps_gpio = PPS_GPIO_PIN;
    }
    ESP_LOGI(GPS_TAG, "Initializing MCPWM hardware capture on GPIO %d", s_pps_gpio);

    // Configure PPS GPIO as floating input to avoid loading open-drain outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_pps_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    mcpwm_capture_timer_config_t cap_timer_config = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = 1000000, // 1 MHz capture resolution
    };

    esp_err_t ret = mcpwm_new_capture_timer(&cap_timer_config, &cap_timer);

    if(ret != ESP_OK) {
        ESP_LOGE(GPS_TAG, "Failed to create a capture timer: %s", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_capture_channel_config_t cap_chan_config = {
        .gpio_num= s_pps_gpio,
        .prescale = 1,
        .flags.neg_edge = false,
        .flags.pos_edge = true,
        .flags.pull_up = false,
    };

    ret = mcpwm_new_capture_channel(cap_timer, &cap_chan_config, &cap_channel);
    if(ret != ESP_OK){
        ESP_LOGE(GPS_TAG, "Failed to create a capture channel: %s", esp_err_to_name(ret));
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = pps_hw_capture_callback,
    };

    ret = mcpwm_capture_channel_register_event_callbacks(cap_channel, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(GPS_TAG, "Failed to register capture callbacks: %s", esp_err_to_name(ret));
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    // enable capture channel
    ret = mcpwm_capture_channel_enable(cap_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(GPS_TAG, "Failed to enable capture channel: %s", esp_err_to_name(ret));
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    // enable and start capture timer
    ret = mcpwm_capture_timer_enable(cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(GPS_TAG, "Failed to enable capture timer: %s", esp_err_to_name(ret));
        mcpwm_capture_channel_disable(cap_channel);
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    ret = mcpwm_capture_timer_start(cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(GPS_TAG, "Failed to start capture timer: %s", esp_err_to_name(ret));
        mcpwm_capture_timer_disable(cap_timer);
        mcpwm_capture_channel_disable(cap_channel);
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    ESP_LOGI(GPS_TAG, "MCPWM hardware capture initialized on GPIO %d", s_pps_gpio);
    return ESP_OK;
}

void gps_init_pps_capture(int pps_gpio) {
    s_pps_gpio = pps_gpio; // use runtime-selected PPS GPIO
    esp_err_t r = init_hardware_pps_capture();
    if (r != ESP_OK) {
        ESP_LOGE(GPS_TAG, "gps_init_pps_capture failed: %s", esp_err_to_name(r));
    }
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

    // Temperature field omitted (no DS3231)
    char temp_field[16] = "";

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

    // Set UART2 TX/RX pins from configuration macros in gps.h
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uint32_t actual_baud = 0;
    uart_get_baudrate(UART_NUM_2, &actual_baud);
    ESP_LOGI(GPS_TAG, "UART2 configured: TX=%d RX=%d baud=%u", (int)GPS_UART_TX_PIN, (int)GPS_UART_RX_PIN, (unsigned)actual_baud);
    
    
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
    uint64_t last_stats_time = 0;
    uint32_t stats_bytes = 0;
    uint32_t stats_dollars = 0;

    while (1) {
    int len = uart_read_bytes(UART_NUM_2, uart_buffer, uart_buffer_size - 1, pdMS_TO_TICKS(GPS_UART_TIMEOUT_MS));

    if (len > 0) {
        stats_bytes += (uint32_t)len;
            for (int i = 0; i < len; i++) {
                char c = uart_buffer[i];
                if (c == '$') {
            stats_dollars++;
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
                                    s_last_gprmc_status = status;
                                    s_last_gprmc_time_us = esp_timer_get_time();
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

            sync_timestamp_us = esp_timer_get_time();
            // Anchor XTAL 32 kHz slow-clock time to UTC at this PPS boundary
            epoch_sync_us = (uint64_t)pending_sync_target_sec * 1000000ULL;
            rtc32k_sync_us = esp_rtc_get_time_us();
            rtc32k_inited = true;
            // latch epoch seconds at the PPS edge we just synced to
            last_pps_epoch_sec = pending_sync_target_sec;
            // Save anchor in RTC memory (for restore after deep sleep)
            s_time_anchor.epoch_sec_at_sync = last_pps_epoch_sec;
            s_time_anchor.rtc32k_time_at_sync = rtc32k_sync_us;
            s_time_anchor.crc32 = 0;
            s_time_anchor.crc32 = crc32_simple(&s_time_anchor, sizeof(s_time_anchor) - sizeof(uint32_t));
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

// ---- Debug helpers implementation ----
uint64_t gps_get_pps_count(void) { return pps_count; }
uint64_t gps_get_total_captures(void) { return total_captures; }
uint32_t gps_get_last_capture_age_ms(void) {
    uint64_t now = esp_timer_get_time();
    uint64_t age_us = (last_callback_time > 0) ? (now - last_callback_time) : UINT64_MAX;
    return (age_us == UINT64_MAX) ? 0xFFFFFFFFu : (uint32_t)(age_us / 1000ULL);
}
bool gps_get_pending_sync(void) { return pending_sync; }
bool gps_get_time_synchronized(void) { return time_synchronized; }
char gps_get_last_gprmc_status(void) { return s_last_gprmc_status; }
uint32_t gps_get_last_gprmc_age_ms(void) {
    uint64_t now = esp_timer_get_time();
    uint64_t age_us = (s_last_gprmc_time_us > 0) ? (now - s_last_gprmc_time_us) : UINT64_MAX;
    return (age_us == UINT64_MAX) ? 0xFFFFFFFFu : (uint32_t)(age_us / 1000ULL);
}
void gps_debug_dump_status(void) {
    ESP_LOGI(GPS_TAG, "DBG: PPS=%llu, cap_total=%llu, cap_age_ms=%u, pending_sync=%d, synced=%d, GPRMC_stat=%c, gprmc_age_ms=%u",
        (unsigned long long)pps_count,
        (unsigned long long)total_captures,
        gps_get_last_capture_age_ms(),
        (int)pending_sync,
        (int)time_synchronized,
        s_last_gprmc_status,
        gps_get_last_gprmc_age_ms());
}

// ---- PPS timing helpers implementation ----
uint64_t gps_get_last_pps_edge_time_us(void) { return last_pps_edge_time_us; }
uint64_t gps_get_last_pps_epoch_sec(void) { return last_pps_epoch_sec; }

bool gps_wait_for_next_pps(uint32_t timeout_ms, uint64_t* out_epoch_sec, uint64_t* out_edge_time_us) {
    uint64_t start = esp_timer_get_time();
    uint64_t start_pps = pps_count;
    while ((esp_timer_get_time() - start) < ((uint64_t)timeout_ms * 1000ULL)) {
        if (pps_count > start_pps) {
            if (out_edge_time_us) *out_edge_time_us = last_pps_edge_time_us;
            if (out_epoch_sec) *out_epoch_sec = last_pps_epoch_sec ? last_pps_epoch_sec : 0ULL;
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ---- Retained time anchor helpers ----
bool gps_has_saved_time_anchor(void) {
    time_anchor_t tmp = s_time_anchor;
    uint32_t c = crc32_simple(&tmp, sizeof(tmp) - sizeof(uint32_t));
    if (tmp.crc32 == c && tmp.epoch_sec_at_sync != 0 && tmp.rtc32k_time_at_sync != 0) {
        return true;
    }
    return false;
}

void gps_clear_saved_time_anchor(void) {
    s_time_anchor.epoch_sec_at_sync = 0;
    s_time_anchor.rtc32k_time_at_sync = 0;
    s_time_anchor.crc32 = 0;
}

bool gps_restore_time_from_anchor_and_pps(uint32_t pps_timeout_ms) {
    if (!gps_has_saved_time_anchor()) return false;
    // Wait for a PPS, then reconstruct epoch seconds without NMEA
    uint64_t edge_epoch = 0, edge_us = 0;
    if (!gps_wait_for_next_pps(pps_timeout_ms, &edge_epoch, &edge_us)) return false;

    // Compute how many seconds elapsed since the saved anchor, using RTC slow clock
    uint64_t now_rtc_us = esp_rtc_get_time_us();
    uint64_t delta_us = (now_rtc_us >= s_time_anchor.rtc32k_time_at_sync) ? (now_rtc_us - s_time_anchor.rtc32k_time_at_sync) : 0ULL;
    uint64_t elapsed_sec = delta_us / 1000000ULL; // coarse: 1-second resolution using PPS for alignment
    uint64_t new_epoch_sec = s_time_anchor.epoch_sec_at_sync + elapsed_sec;

    // Align settimeofday to the observed PPS edge
    struct timeval tv;
    tv.tv_sec = (time_t)new_epoch_sec;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    // Update local sync state like the normal path
    synchronized_time = *gmtime((time_t*)&tv.tv_sec);
    sync_timestamp_us = esp_timer_get_time();
    epoch_sync_us = (uint64_t)new_epoch_sec * 1000000ULL;
    rtc32k_sync_us = esp_rtc_get_time_us();
    rtc32k_inited = true;
    last_pps_epoch_sec = new_epoch_sec;
    time_synchronized = true;
    pending_sync = false;
    pps_count = 0;
    ESP_LOGI(GPS_TAG, "Time restored from anchor at PPS: %04d-%02d-%02d %02d:%02d:%02d UTC",
             synchronized_time.tm_year + 1900, synchronized_time.tm_mon + 1, synchronized_time.tm_mday,
             synchronized_time.tm_hour, synchronized_time.tm_min, synchronized_time.tm_sec);
    return true;
}
