#include "sdcard.h"
#include <stdio.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_defs.h>
#include "driver/gpio.h"

#define PIN_NUM_MOSI  GPIO_NUM_38
#define PIN_NUM_CS    GPIO_NUM_39
#define PIN_NUM_CLK   GPIO_NUM_40
#define PIN_NUM_MISO  GPIO_NUM_41

static const char *SD_TAG = "SD Card";

bool file_exists(const char *filename) {
    struct stat st; return stat(filename, &st) == 0;
}

int sd_write_index(const char *path) {
    FILE *file = fopen(path, "r");
    if (!file) {
        file = fopen(path, "w");
        if (!file) { ESP_LOGE(SD_TAG, "Failed to create %s", path); return 0; }
        fputs("1", file); fclose(file); return 1;
    }
    int current_number = 0; fscanf(file, "%d", &current_number); fclose(file);
    FILE *wf = fopen(path, "w");
    if (!wf) { ESP_LOGE(SD_TAG, "Failed to update %s", path); return current_number; }
    fprintf(wf, "%d", current_number + 1); fclose(wf); return current_number;
}

esp_err_t sd_write_file_header(const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) { ESP_LOGE(SD_TAG, "Header open failed: %s", path); return ESP_FAIL; }
    else{ ESP_LOGI(SD_TAG, "Header open succeeded: %s", path);}    
    fputs("Timestamp, Log", f);
    fclose(f); return ESP_OK;
}

esp_err_t sd_append_line(const char *path, const char *line) {
    FILE *f = fopen(path, "a");
    if (!f) { ESP_LOGE(SD_TAG, "Append open failed: %s", path); return ESP_FAIL; }
    size_t len = strlen(line);
    fwrite(line, 1, len, f);
    if (len == 0 || line[len-1] != '\n') fputc('\n', f);
    fclose(f); return ESP_OK;
}

void sdcardinitialize(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    if (spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA) != ESP_OK) {
        ESP_LOGE(SD_TAG, "SPI bus init failed"); return; }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS; slot_config.host_id = host.slot;
    ESP_LOGI(SD_TAG, "Mounting SD card at %s", MOUNT_POINT);
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) ESP_LOGE(SD_TAG, "Mount failed (format_if_mount_failed=%d)", mount_config.format_if_mount_failed);
        else ESP_LOGE(SD_TAG, "Card init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(SD_TAG, "SD mounted");
    sdmmc_card_print_info(stdout, card);
}
