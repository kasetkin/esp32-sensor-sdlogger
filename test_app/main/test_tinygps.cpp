#include "unity.h"
#include "TinyGPSPlus.h"
#include <cmath>
#include <numbers>
#include <array>
#include <string_view>
#include <esp_timer.h>
#include <esp_log.h>

// ── radians() ────────────────────────────────────────────────────────────────

static void test_tinygps_radians_zero(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, TinyGPSPlus::radians(0.0));
}

static void test_tinygps_radians_180(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, std::numbers::pi, TinyGPSPlus::radians(180.0));
}

static void test_tinygps_radians_90(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, std::numbers::pi / 2.0, TinyGPSPlus::radians(90.0));
}

static void test_tinygps_radians_360(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.0 * std::numbers::pi, TinyGPSPlus::radians(360.0));
}

static void test_tinygps_radians_negative(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -std::numbers::pi / 2.0, TinyGPSPlus::radians(-90.0));
}

// ── degrees() ────────────────────────────────────────────────────────────────

static void test_tinygps_degrees_zero(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, TinyGPSPlus::degrees(0.0));
}

static void test_tinygps_degrees_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 180.0, TinyGPSPlus::degrees(std::numbers::pi));
}

static void test_tinygps_degrees_half_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 90.0, TinyGPSPlus::degrees(std::numbers::pi / 2.0));
}

static void test_tinygps_degrees_two_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 360.0, TinyGPSPlus::degrees(2.0 * std::numbers::pi));
}

static void test_tinygps_degrees_negative(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -180.0, TinyGPSPlus::degrees(-std::numbers::pi));
}

// ── round-trip ────────────────────────────────────────────────────────────────

static void test_tinygps_roundtrip_degrees_radians(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 45.0, TinyGPSPlus::degrees(TinyGPSPlus::radians(45.0)));
}

static void test_parseDecimal_benchmark(void)
{
  static constexpr std::array testValues{
    std::string_view{"5.4"},    std::string_view{"1.0"},     std::string_view{"0.09"},
    std::string_view{"-3.7"},   std::string_view{"100.25"},  std::string_view{"999.99"},
    std::string_view{"5"},      std::string_view{"0"},       std::string_view{"-100"}
  };
  constexpr int ITERATIONS = 1000;
  volatile int32_t sink = 0;

  const int64_t t0 = esp_timer_get_time();
  for (int i = 0; i < ITERATIONS; ++i)
    for (const auto sv : testValues)
      sink += TinyGPSPlus::parseDecimal(sv);
  const int64_t t1 = esp_timer_get_time();

  const int64_t t2 = esp_timer_get_time();
  for (int i = 0; i < ITERATIONS; ++i)
    for (const auto sv : testValues)
      sink += TinyGPSPlus::parseDecimalFloat(sv);
  const int64_t t3 = esp_timer_get_time();

  const int64_t t4 = esp_timer_get_time();
  for (int i = 0; i < ITERATIONS; ++i)
    for (const auto sv : testValues)
      sink += TinyGPSPlus::parseDecimalOld(sv);
  const int64_t t5 = esp_timer_get_time();

  const int totalCalls = ITERATIONS * static_cast<int>(testValues.size());
  ESP_LOGI("perf-decimal", "parseDecimal      : total %lld us, mean %.3f us/call",
           t1 - t0, static_cast<float>(t1 - t0) / totalCalls);
  ESP_LOGI("perf-decimal", "parseDecimalFloat : total %lld us, mean %.3f us/call",
           t3 - t2, static_cast<float>(t3 - t2) / totalCalls);
  ESP_LOGI("perf-decimal", "parseDecimalOld   : total %lld us, mean %.3f us/call",
           t5 - t4, static_cast<float>(t5 - t4) / totalCalls);
  (void)sink;
}

void run_tinygps_tests(void)
{
    RUN_TEST(test_tinygps_radians_zero);
    RUN_TEST(test_tinygps_radians_180);
    RUN_TEST(test_tinygps_radians_90);
    RUN_TEST(test_tinygps_radians_360);
    RUN_TEST(test_tinygps_radians_negative);
    RUN_TEST(test_tinygps_degrees_zero);
    RUN_TEST(test_tinygps_degrees_pi);
    RUN_TEST(test_tinygps_degrees_half_pi);
    RUN_TEST(test_tinygps_degrees_two_pi);
    RUN_TEST(test_tinygps_degrees_negative);
    RUN_TEST(test_tinygps_roundtrip_degrees_radians);
    RUN_TEST(test_parseDecimal_benchmark);
}
