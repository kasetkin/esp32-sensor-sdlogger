set(MAIN_SRC "../../main")

set(srcs
    test_runner.cpp
    test_logger.cpp
    test_sensors.cpp
    test_gps.cpp
    test_unicore.cpp
    test_ppp.cpp
    test_crc.cpp
    test_sdcard.cpp
    test_geodistance.cpp
    test_tinygps.cpp
    ${MAIN_SRC}/sensorstask.cpp
    ${MAIN_SRC}/loggertask.cpp
    ${MAIN_SRC}/gpstask.cpp
    ${MAIN_SRC}/sdcard.cpp
    ${MAIN_SRC}/common_utils.cpp
    ${MAIN_SRC}/errortask.cpp
    ${MAIN_SRC}/bleservertask.cpp
    ${MAIN_SRC}/fake_nmea.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." ${MAIN_SRC}
                       REQUIRES sht3x fatfs sdmmc nvs_flash esp_timer esp_driver_gpio
                                esp_driver_uart esp_adc bt unity
                       PRIV_REQUIRES TinyGPSPlus
                       WHOLE_ARCHIVE)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 26)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD_REQUIRED ON)