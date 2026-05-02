#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern void run_logger_tests(void);
extern void run_sensors_tests(void);
extern void run_gps_tests(void);
extern void run_unicore_tests(void);
extern void run_ppp_tests(void);
extern void run_crc_tests(void);
extern void run_sdcard_tests(void);
extern void run_geodistance_tests(void);

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    run_logger_tests();
    run_sensors_tests();
    run_gps_tests();
    run_unicore_tests();
    run_ppp_tests();
    run_crc_tests();
    run_sdcard_tests();
    run_geodistance_tests();
    UNITY_END();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
