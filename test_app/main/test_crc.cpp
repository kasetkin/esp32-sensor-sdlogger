#include "unity.h"
#include "unicore.h"
#include <cstring>

// ── pushByte32BitCrc ──────────────────────────────────────────────────────────

static void test_crc_init_value_is_zero(void)
{
    // no bytes pushed — checksum must start at 0 (callers initialise to 0)
    uint32_t checksum = 0;
    TEST_ASSERT_EQUAL_UINT32(0, checksum);
}

static void test_crc_zero_byte_stays_zero(void)
{
    // aulCrcTable[0] = 0 → result stays 0
    uint32_t checksum = 0;
    pushByte32BitCrc(0x00, checksum);
    TEST_ASSERT_EQUAL_UINT32(0, checksum);
}

static void test_crc_single_byte_0x01(void)
{
    // one step: aulCrcTable[(0 ^ 0x01) & 0xFF] ^ (0 >> 8) = aulCrcTable[1]
    uint32_t checksum = 0;
    pushByte32BitCrc(0x01, checksum);
    TEST_ASSERT_EQUAL_UINT32(aulCrcTable[1], checksum);
}

static void test_crc_single_byte_0xFF(void)
{
    // one step: aulCrcTable[0xFF ^ 0] = aulCrcTable[255]
    uint32_t checksum = 0;
    pushByte32BitCrc(0xFF, checksum);
    TEST_ASSERT_EQUAL_UINT32(aulCrcTable[255], checksum);
}

static void test_crc_all_zeros_stays_zero(void)
{
    // aulCrcTable[0]=0 so checksum stays 0 at every step
    uint32_t checksum = 0;
    for (int i = 0; i < 4; ++i)
        pushByte32BitCrc(0x00, checksum);
    TEST_ASSERT_EQUAL_UINT32(0, checksum);
}

static void test_crc_two_bytes_known_value(void)
{
    // pre-computed: push {0x01,0x02} → 0xF715506D
    uint32_t checksum = 0;
    pushByte32BitCrc(0x01, checksum);
    pushByte32BitCrc(0x02, checksum);
    TEST_ASSERT_EQUAL_UINT32(0xF715506DUL, checksum);
}

static void test_crc_order_matters(void)
{
    uint32_t crc_ab = 0;
    pushByte32BitCrc(0x01, crc_ab);
    pushByte32BitCrc(0x02, crc_ab);  // 0xF715506D

    uint32_t crc_ba = 0;
    pushByte32BitCrc(0x02, crc_ba);
    pushByte32BitCrc(0x01, crc_ba);  // 0x45315214

    TEST_ASSERT_TRUE(crc_ab != crc_ba);
}

static void test_crc_nonzero_init_differs(void)
{
    uint32_t from_zero = 0;
    pushByte32BitCrc(0x01, from_zero);

    uint32_t from_ones = 0xFFFFFFFFUL;
    pushByte32BitCrc(0x01, from_ones);

    TEST_ASSERT_TRUE(from_zero != from_ones);
}

static void test_crc_long_input_known_value(void)
{
    // 46-byte string; expected value captured from device output
    static const char input[] = "The quick brown fox jumps over the lazy dog!!!";
    uint32_t checksum = 0;
    for (size_t i = 0; i < strlen(input); ++i)
        pushByte32BitCrc(static_cast<uint8_t>(input[i]), checksum);
    TEST_ASSERT_EQUAL_UINT32(0x4DB495E6UL, checksum);
}

void run_crc_tests(void)
{
    RUN_TEST(test_crc_init_value_is_zero);
    RUN_TEST(test_crc_zero_byte_stays_zero);
    RUN_TEST(test_crc_single_byte_0x01);
    RUN_TEST(test_crc_single_byte_0xFF);
    RUN_TEST(test_crc_all_zeros_stays_zero);
    RUN_TEST(test_crc_two_bytes_known_value);
    RUN_TEST(test_crc_order_matters);
    RUN_TEST(test_crc_nonzero_init_differs);
    RUN_TEST(test_crc_long_input_known_value);
}
