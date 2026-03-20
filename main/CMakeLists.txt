set(srcs main.cpp sdcard.cpp common_utils.cpp)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES fatfs sd_card nvs_flash esp_timer esp_driver_gpio esp_driver_uart
                       WHOLE_ARCHIVE)
