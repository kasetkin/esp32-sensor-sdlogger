set(srcs 
  main.cpp 
  sdcard.cpp 
  common_utils.cpp 
  gpstask.cpp
  sensorstask.cpp
  loggertask.cpp
  errortask.cpp
  bleservertask.cpp
  # fake_nmea.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES sht3x fatfs sdmmc nvs_flash esp_timer esp_driver_gpio esp_driver_uart esp_adc bt
                       WHOLE_ARCHIVE)

target_link_libraries(${COMPONENT_LIB} PRIVATE tinygpsplus)
