set(srcs 
  main.cpp 
  sdcard.cpp 
  common_utils.cpp 
  gpstask.cpp
  sensorstask.cpp
  loggertask.cpp
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES fatfs sdmmc nvs_flash esp_timer esp_driver_gpio esp_driver_uart esp_adc
                       WHOLE_ARCHIVE)

target_link_libraries(${COMPONENT_LIB} PRIVATE tinygpsplus)