#include "unity.h"
#include "loggertask.h"
#include <sys/time.h>

static void test_pads_single_digit_to_2(void)
{
    TEST_ASSERT_EQUAL_STRING("05", LoggerTask::toStringWithZeros(5, 2).c_str());
}

static void test_zero_pads_to_2(void)
{
    TEST_ASSERT_EQUAL_STRING("00", LoggerTask::toStringWithZeros(0, 2).c_str());
}

static void test_pads_to_4_digits(void)
{
    TEST_ASSERT_EQUAL_STRING("0042", LoggerTask::toStringWithZeros(42, 4).c_str());
}

static void test_exact_length_no_padding(void)
{
    TEST_ASSERT_EQUAL_STRING("12", LoggerTask::toStringWithZeros(12, 2).c_str());
}

static void test_longer_than_requested_no_truncation(void)
{
    TEST_ASSERT_EQUAL_STRING("123", LoggerTask::toStringWithZeros(123, 2).c_str());
}

static void test_single_digit_no_padding_needed(void)
{
    TEST_ASSERT_EQUAL_STRING("7", LoggerTask::toStringWithZeros(7, 1).c_str());
}

// ── generateFilename ──────────────────────────────────────────────────────────

static void test_generate_filename_zero_time(void)
{
    struct timeval tv = {0, 0};
    settimeofday(&tv, nullptr);
    TEST_ASSERT_EQUAL_STRING("NO-DATE-FILE", LoggerTask::generateFilename().c_str());
}

static void test_generate_filename_known_date(void)
{
    // 1700000000 → 2023-11-14 22:13:20 UTC
    struct timeval tv = {1700000000, 0};
    settimeofday(&tv, nullptr);
    TEST_ASSERT_EQUAL_STRING("2023-11-14-sdlogger_UM980",
                             LoggerTask::generateFilename().c_str());
}

static void test_generate_filename_zero_padded_month_day(void)
{
    // 1672876800 → 2023-01-05 00:00:00 UTC — verifies month and day are zero-padded
    struct timeval tv = {1672876800, 0};
    settimeofday(&tv, nullptr);
    TEST_ASSERT_EQUAL_STRING("2023-01-05-sdlogger_UM980",
                             LoggerTask::generateFilename().c_str());
}

void run_logger_tests(void)
{
    RUN_TEST(test_pads_single_digit_to_2);
    RUN_TEST(test_zero_pads_to_2);
    RUN_TEST(test_pads_to_4_digits);
    RUN_TEST(test_exact_length_no_padding);
    RUN_TEST(test_longer_than_requested_no_truncation);
    RUN_TEST(test_single_digit_no_padding_needed);
    RUN_TEST(test_generate_filename_zero_time);
    RUN_TEST(test_generate_filename_known_date);
    RUN_TEST(test_generate_filename_zero_padded_month_day);
}
