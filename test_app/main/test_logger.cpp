#include "unity.h"
#include "loggertask.h"
#include <sys/time.h>

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
    RUN_TEST(test_generate_filename_zero_time);
    RUN_TEST(test_generate_filename_known_date);
    RUN_TEST(test_generate_filename_zero_padded_month_day);
}
