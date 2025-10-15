idf_component_register(
    SRCS "sdcard.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_vfs_fat sdmmc
)
