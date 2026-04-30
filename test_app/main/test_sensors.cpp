#include "unity.h"
#include <cmath>
#include <cstring>
#include "sensorstask.h"
#include <sht3x.h>

// ── SensorsValues::toTelemetryRoundedString ──────────────────────────────────

static void test_telemetry_rounded_3_digits(void)
{
    TEST_ASSERT_EQUAL_STRING("25.123",
        SensorsValues::toTelemetryRoundedString(25.123456f).c_str());
}

static void test_telemetry_rounded_whole_number(void)
{
    TEST_ASSERT_EQUAL_STRING("100",
        SensorsValues::toTelemetryRoundedString(100.0f).c_str());
}

static void test_telemetry_rounded_trailing_zeros(void)
{
    TEST_ASSERT_EQUAL_STRING("1.5",
        SensorsValues::toTelemetryRoundedString(1.5f).c_str());
}

// ── SensorsValues::toString ──────────────────────────────────────────────────

static void test_to_string_empty_defaults(void)
{
    SensorsValues v;
    TEST_ASSERT_EQUAL_STRING("", v.toString().c_str());
}

static void test_to_string_includes_batvolt(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    TEST_ASSERT_TRUE(v.toString().find("BATVOLT;3700;") != std::string::npos);
}

static void test_to_string_skips_batperc_zero(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    v.batteryPercent = 0;
    TEST_ASSERT_TRUE(v.toString().find("BATPERC") == std::string::npos);
}

static void test_to_string_includes_both_batvolt_batperc(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    v.batteryPercent = 50;
    TEST_ASSERT_EQUAL_STRING("BATVOLT;3700;BATPERC;50;", v.toString().c_str());
}

static void test_to_string_includes_temp(void)
{
    SensorsValues v;
    v.envTemperature = 25.0f;
    TEST_ASSERT_EQUAL_STRING("TEMP;25;", v.toString().c_str());
}

static void test_to_string_includes_humid(void)
{
    SensorsValues v;
    v.envHumidity = 60.0f;
    TEST_ASSERT_EQUAL_STRING("HUMID;60;", v.toString().c_str());
}

static void test_to_string_omits_nan_fields(void)
{
    SensorsValues v;
    v.batteryPercent = 75;
    v.batteryVoltageMilliV = 4000;
    const std::string result = v.toString();
    TEST_ASSERT_TRUE(result.find("TEMP")  == std::string::npos);
    TEST_ASSERT_TRUE(result.find("HUMID") == std::string::npos);
    TEST_ASSERT_TRUE(result.find("PRESS") == std::string::npos);
}

// ── SensorsTask::convertVoltageToPercent ─────────────────────────────────────

static void test_convert_voltage_max_gives_100(void)
{
    TEST_ASSERT_EQUAL_INT(100, SensorsTask::convertVoltageToPercent(4200));
}

static void test_convert_voltage_min_gives_0(void)
{
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(3300));
}

static void test_convert_voltage_midpoint_gives_50(void)
{
    TEST_ASSERT_EQUAL_INT(50, SensorsTask::convertVoltageToPercent(3750));
}

static void test_convert_voltage_clamps_below_min(void)
{
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(2000));
}

static void test_convert_voltage_clamps_above_max(void)
{
    TEST_ASSERT_EQUAL_INT(100, SensorsTask::convertVoltageToPercent(5000));
}

// ── Hardware: ADC calibration + SHT3x ────────────────────────────────────────

static SensorsTask g_sensorsTask;
static bool g_sensorsInitialized = false;

static void ensureSensorsInit(void)
{
    if (!g_sensorsInitialized) {
        const esp_err_t err = g_sensorsTask.init();
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "SensorsTask::init() failed - check ADC wiring and I2C/SHT3x connection");
        g_sensorsInitialized = true;
    }
}

static void test_hw_sensors_init(void)
{
    ensureSensorsInit();
}

static void test_hw_sht3x_values_in_range(void)
{
    ensureSensorsInit();

    float temperature = 0.0f;
    float humidity    = 0.0f;
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, g_sensorsTask.readEnvironment(temperature, humidity),
                              "readEnvironment() failed");

    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(-20.0f, temperature);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(80.0f, temperature);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, humidity);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(100.0f, humidity);
}

void run_sensors_tests(void)
{
    RUN_TEST(test_telemetry_rounded_3_digits);
    RUN_TEST(test_telemetry_rounded_whole_number);
    RUN_TEST(test_telemetry_rounded_trailing_zeros);
    RUN_TEST(test_to_string_empty_defaults);
    RUN_TEST(test_to_string_includes_batvolt);
    RUN_TEST(test_to_string_skips_batperc_zero);
    RUN_TEST(test_to_string_includes_both_batvolt_batperc);
    RUN_TEST(test_to_string_includes_temp);
    RUN_TEST(test_to_string_includes_humid);
    RUN_TEST(test_to_string_omits_nan_fields);
    RUN_TEST(test_convert_voltage_max_gives_100);
    RUN_TEST(test_convert_voltage_min_gives_0);
    RUN_TEST(test_convert_voltage_midpoint_gives_50);
    RUN_TEST(test_convert_voltage_clamps_below_min);
    RUN_TEST(test_convert_voltage_clamps_above_max);
    RUN_TEST(test_hw_sensors_init);
    RUN_TEST(test_hw_sht3x_values_in_range);
}
