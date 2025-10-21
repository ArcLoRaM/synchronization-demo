#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOUNT_POINT "/sdcard"

bool file_exists(const char *filename);
int sd_write_index(const char *path);
esp_err_t sd_write_file_header(const char *path);
esp_err_t sd_append_line(const char *path, const char *line);
void sdcardinitialize(void);

#ifdef __cplusplus
}
#endif
