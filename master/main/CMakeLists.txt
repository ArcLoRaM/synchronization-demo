idf_component_register(SRCS "main.cpp"
                    REQUIRES spi_flash RadioLib oled_display gps sdcard esp_driver_mcpwm
                    INCLUDE_DIRS ".")
