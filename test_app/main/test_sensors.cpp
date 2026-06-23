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

static void test_telemetry_negative_value(void)
{
    // -40.125 is exactly representable in float (= -(32+8+1/8))
    TEST_ASSERT_EQUAL_STRING("-40.125",
        SensorsValues::toTelemetryRoundedString(-40.125f).c_str());
}

static void test_telemetry_large_whole_number(void)
{
    TEST_ASSERT_EQUAL_STRING("125",
        SensorsValues::toTelemetryRoundedString(125.0f).c_str());
}

// ── SensorsValues::toString ──────────────────────────────────────────────────

static void test_to_string_empty_defaults(void)
{
    SensorsValues v;
    TEST_ASSERT_EQUAL_STRING("", v.toTelemetryString().c_str());
}

static void test_to_string_includes_batvolt(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    TEST_ASSERT_TRUE(v.toTelemetryString().contains("BATVOLT;3700;"));
}

static void test_to_string_skips_batperc_zero(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    // batteryPercent left as nullopt — no value assigned
    TEST_ASSERT_FALSE(v.toTelemetryString().contains("BATPERC"));
}

static void test_to_string_includes_both_batvolt_batperc(void)
{
    SensorsValues v;
    v.batteryVoltageMilliV = 3700;
    v.batteryPercent = 50;
    TEST_ASSERT_EQUAL_STRING("BATVOLT;3700;BATPERC;50;", v.toTelemetryString().c_str());
}

static void test_to_string_includes_temp(void)
{
    SensorsValues v;
    v.envTemperature = 25.0f;
    TEST_ASSERT_EQUAL_STRING("TEMP;25;", v.toTelemetryString().c_str());
}

static void test_to_string_includes_humid(void)
{
    SensorsValues v;
    v.envHumidity = 60.0f;
    TEST_ASSERT_EQUAL_STRING("HUMID;60;", v.toTelemetryString().c_str());
}

static void test_to_string_omits_nan_fields(void)
{
    SensorsValues v;
    v.batteryPercent = 75;
    v.batteryVoltageMilliV = 4000;
    const std::string result = v.toTelemetryString();
    TEST_ASSERT_FALSE(result.contains("TEMP"));
    TEST_ASSERT_FALSE(result.contains("HUMID"));
    TEST_ASSERT_FALSE(result.contains("PRESS"));
}

static void test_to_string_min_temperature(void)
{
    SensorsValues v;
    v.envTemperature = -40.0f;  // SHT3x lower limit
    TEST_ASSERT_EQUAL_STRING("TEMP;-40;", v.toTelemetryString().c_str());
}

static void test_to_string_max_temperature(void)
{
    SensorsValues v;
    v.envTemperature = 125.0f;  // SHT3x upper limit
    TEST_ASSERT_EQUAL_STRING("TEMP;125;", v.toTelemetryString().c_str());
}

static void test_to_string_max_humidity(void)
{
    SensorsValues v;
    v.envHumidity = 100.0f;
    TEST_ASSERT_EQUAL_STRING("HUMID;100;", v.toTelemetryString().c_str());
}

static void test_to_string_zero_humidity(void)
{
    SensorsValues v;
    v.envHumidity = 0.0f;  // not NaN — must be included in output
    TEST_ASSERT_EQUAL_STRING("HUMID;0;", v.toTelemetryString().c_str());
}

// ── SensorsTask::convertVoltageToPercent ─────────────────────────────────────

static constexpr int V_MAX = static_cast<int>(SensorsTask::MAX_VOLTAGE);
static constexpr int V_MIN = static_cast<int>(SensorsTask::MIN_VOLTAGE);
static constexpr int V_MID = static_cast<int>((SensorsTask::MIN_VOLTAGE + SensorsTask::MAX_VOLTAGE) / 2.0);

static void test_convert_voltage_max_gives_100(void)
{
    TEST_ASSERT_EQUAL_INT(100, SensorsTask::convertVoltageToPercent(V_MAX));
}

static void test_convert_voltage_min_gives_0(void)
{
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(V_MIN));
}

static void test_convert_voltage_midpoint_gives_50(void)
{
    TEST_ASSERT_EQUAL_INT(50, SensorsTask::convertVoltageToPercent(V_MID));
}

static void test_convert_voltage_clamps_below_min(void)
{
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(V_MIN - 500));
}

static void test_convert_voltage_clamps_above_max(void)
{
    TEST_ASSERT_EQUAL_INT(100, SensorsTask::convertVoltageToPercent(V_MAX + 500));
}

static void test_convert_voltage_just_below_min_clamps_to_0(void)
{
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(V_MIN - 1));
}

static void test_convert_voltage_just_above_min_truncates_to_0(void)
{
    // 1/(MAX-MIN)*100 < 1 for any realistic range → truncates to 0
    TEST_ASSERT_EQUAL_INT(0, SensorsTask::convertVoltageToPercent(V_MIN + 1));
}

static void test_convert_voltage_just_below_max_gives_99(void)
{
    // (MAX-1-MIN)/(MAX-MIN)*100 = 100 - 100/DELTA, which truncates to 99 for any DELTA > 1
    TEST_ASSERT_EQUAL_INT(99, SensorsTask::convertVoltageToPercent(V_MAX - 1));
}

static void test_convert_voltage_just_above_max_clamps_to_100(void)
{
    TEST_ASSERT_EQUAL_INT(100, SensorsTask::convertVoltageToPercent(V_MAX + 1));
}

// ── Hardware: ADC calibration + SHT3x ────────────────────────────────────────

static SensorsTask g_sensorsTask;
static bool g_sensorsInitialized = false;

static void ensureSensorsInit(void)
{
    if (!g_sensorsInitialized) {
        const esp_err_t err = g_sensorsTask.init(false);  // match main.cpp: ADC reading disabled
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
    RUN_TEST(test_telemetry_negative_value);
    RUN_TEST(test_telemetry_large_whole_number);
    RUN_TEST(test_to_string_empty_defaults);
    RUN_TEST(test_to_string_includes_batvolt);
    RUN_TEST(test_to_string_skips_batperc_zero);
    RUN_TEST(test_to_string_includes_both_batvolt_batperc);
    RUN_TEST(test_to_string_includes_temp);
    RUN_TEST(test_to_string_includes_humid);
    RUN_TEST(test_to_string_omits_nan_fields);
    RUN_TEST(test_to_string_min_temperature);
    RUN_TEST(test_to_string_max_temperature);
    RUN_TEST(test_to_string_max_humidity);
    RUN_TEST(test_to_string_zero_humidity);
    RUN_TEST(test_convert_voltage_max_gives_100);
    RUN_TEST(test_convert_voltage_min_gives_0);
    RUN_TEST(test_convert_voltage_midpoint_gives_50);
    RUN_TEST(test_convert_voltage_clamps_below_min);
    RUN_TEST(test_convert_voltage_clamps_above_max);
    RUN_TEST(test_convert_voltage_just_below_min_clamps_to_0);
    RUN_TEST(test_convert_voltage_just_above_min_truncates_to_0);
    RUN_TEST(test_convert_voltage_just_below_max_gives_99);
    RUN_TEST(test_convert_voltage_just_above_max_clamps_to_100);
    RUN_TEST(test_hw_sensors_init);
    RUN_TEST(test_hw_sht3x_values_in_range);
}
