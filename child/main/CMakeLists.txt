idf_component_register(SRCS "main.cpp"
                    REQUIRES spi_flash RadioLib oled_display sdcard esp32-ds3231-1
                    INCLUDE_DIRS ".")
