#include "unity.h"
#include "common_utils.h"

// ── intToStringWithZeros ──────────────────────────────────────────────────────

static void test_pads_single_digit_to_2(void)
{
    TEST_ASSERT_EQUAL_STRING("05", intToStringWithZeros(5, 2).c_str());
}

static void test_zero_pads_to_2(void)
{
    TEST_ASSERT_EQUAL_STRING("00", intToStringWithZeros(0, 2).c_str());
}

static void test_pads_to_4_digits(void)
{
    TEST_ASSERT_EQUAL_STRING("0042", intToStringWithZeros(42, 4).c_str());
}

static void test_exact_length_no_padding(void)
{
    TEST_ASSERT_EQUAL_STRING("12", intToStringWithZeros(12, 2).c_str());
}

static void test_longer_than_requested_no_truncation(void)
{
    TEST_ASSERT_EQUAL_STRING("123", intToStringWithZeros(123, 2).c_str());
}

static void test_single_digit_no_padding_needed(void)
{
    TEST_ASSERT_EQUAL_STRING("7", intToStringWithZeros(7, 1).c_str());
}

void run_common_utils_tests(void)
{
    RUN_TEST(test_pads_single_digit_to_2);
    RUN_TEST(test_zero_pads_to_2);
    RUN_TEST(test_pads_to_4_digits);
    RUN_TEST(test_exact_length_no_padding);
    RUN_TEST(test_longer_than_requested_no_truncation);
    RUN_TEST(test_single_digit_no_padding_needed);
}
