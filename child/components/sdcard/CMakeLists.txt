idf_component_register(
    SRCS "sdcard.c"
    INCLUDE_DIRS "include"
    REQUIRES driver fatfs vfs sdmmc
)
