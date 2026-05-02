#include "unity.h"
#include "TinyGPSPlus.h"
#include <cmath>

// ── radians() ────────────────────────────────────────────────────────────────

static void test_tinygps_radians_zero(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, TinyGPSPlus::radians(0.0));
}

static void test_tinygps_radians_180(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, M_PI, TinyGPSPlus::radians(180.0));
}

static void test_tinygps_radians_90(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, M_PI / 2.0, TinyGPSPlus::radians(90.0));
}

static void test_tinygps_radians_360(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 2.0 * M_PI, TinyGPSPlus::radians(360.0));
}

static void test_tinygps_radians_negative(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -M_PI / 2.0, TinyGPSPlus::radians(-90.0));
}

// ── degrees() ────────────────────────────────────────────────────────────────

static void test_tinygps_degrees_zero(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 0.0, TinyGPSPlus::degrees(0.0));
}

static void test_tinygps_degrees_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 180.0, TinyGPSPlus::degrees(M_PI));
}

static void test_tinygps_degrees_half_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 90.0, TinyGPSPlus::degrees(M_PI / 2.0));
}

static void test_tinygps_degrees_two_pi(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, 360.0, TinyGPSPlus::degrees(2.0 * M_PI));
}

static void test_tinygps_degrees_negative(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-10, -180.0, TinyGPSPlus::degrees(-M_PI));
}

// ── round-trip ────────────────────────────────────────────────────────────────

static void test_tinygps_roundtrip_degrees_radians(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 45.0, TinyGPSPlus::degrees(TinyGPSPlus::radians(45.0)));
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
}
