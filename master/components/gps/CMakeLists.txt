idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp32-ds3231-1
)
