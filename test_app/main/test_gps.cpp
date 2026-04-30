#include "unity.h"
#include <chrono>
#include "gpstask.h"

// ── GpsTask::hasLock ──────────────────────────────────────────────────────────

static void test_has_lock_false_when_invalid(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::Invalid;
    info.fixType = 3;
    TEST_ASSERT_FALSE(GpsTask::hasLock(info));
}

static void test_has_lock_true_gps_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_true_gps_2d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 2;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_true_gps_fixtype_0(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 0;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_false_gps_fixtype_1(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 1;
    TEST_ASSERT_FALSE(GpsTask::hasLock(info));
}

static void test_has_lock_true_rtk_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::RTK;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_true_floatrtk_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::FloatRTK;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

// ── GpsTask::has3DLock ────────────────────────────────────────────────────────

static void test_has_3d_lock_true_gps_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::has3DLock(info));
}

static void test_has_3d_lock_false_gps_2d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 2;
    TEST_ASSERT_FALSE(GpsTask::has3DLock(info));
}

static void test_has_3d_lock_false_fixtype_0(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::GPS;
    info.fixType = 0;
    TEST_ASSERT_FALSE(GpsTask::has3DLock(info));
}

static void test_has_3d_lock_false_invalid(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::Invalid;
    info.fixType = 3;
    TEST_ASSERT_FALSE(GpsTask::has3DLock(info));
}

// ── GpsTask::dopToMeters ──────────────────────────────────────────────────────

static void test_dop_100_gives_1_0(void)
{
    TEST_ASSERT_EQUAL_STRING("1.0", GpsTask::dopToMeters(100).c_str());
}

static void test_dop_150_gives_1_50(void)
{
    TEST_ASSERT_EQUAL_STRING("1.50", GpsTask::dopToMeters(150).c_str());
}

static void test_dop_200_gives_2_0(void)
{
    TEST_ASSERT_EQUAL_STRING("2.0", GpsTask::dopToMeters(200).c_str());
}

static void test_dop_250_gives_2_50(void)
{
    TEST_ASSERT_EQUAL_STRING("2.50", GpsTask::dopToMeters(250).c_str());
}

// ── GpsTask::printGpsGeoInfo ──────────────────────────────────────────────────

static void test_print_gps_geo_info_keys(void)
{
    GpsInfo info;
    info.lat      = 1.0;
    info.lon      = 2.0;
    info.altitude = 10.0;
    info.geoidAlt = 5.0;
    info.gsaPDOP  = 200;
    info.gsaHDOP  = 150;
    info.gsaVDOP  = 250;

    const std::string result = GpsTask::printGpsGeoInfo(info);

    TEST_ASSERT_TRUE(result.find("LAT;1.000000")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("LON;2.000000")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("ALT;10.000000")  != std::string::npos);
    TEST_ASSERT_TRUE(result.find("UNDUL;5.000000") != std::string::npos);
    TEST_ASSERT_TRUE(result.find("PDOP;2.0")       != std::string::npos);
    TEST_ASSERT_TRUE(result.find("HDOP;1.50")      != std::string::npos);
    TEST_ASSERT_TRUE(result.find("VDOP;2.50")      != std::string::npos);
}

// ── GpsTask::printGpsTimeInfo ─────────────────────────────────────────────────

static void test_print_gps_time_empty_without_3d_lock(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 2;
    info.worldTime = std::chrono::system_clock::from_time_t(0);
    TEST_ASSERT_EQUAL_STRING("", GpsTask::printGpsTimeInfo(info).c_str());
}

static void test_print_gps_time_formats_unix_epoch(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.worldTime = std::chrono::system_clock::from_time_t(0);

    const std::string result = GpsTask::printGpsTimeInfo(info);
    TEST_ASSERT_TRUE(result.find("DT;1970-01-01T00:00:00Z") != std::string::npos);
    TEST_ASSERT_TRUE(result.find("GNSSSEC;0")               != std::string::npos);
}

// ── GpsTask::emulateQstarzBinary ─────────────────────────────────────────────

static void test_emulate_qstarz_packet_sizes(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.lat       = 55.7558;
    info.lon       = 37.6173;
    info.altitude  = 150.0;
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);

    TEST_ASSERT_EQUAL(4,  packets.size());
    TEST_ASSERT_EQUAL(20, packets[0].size());
    TEST_ASSERT_EQUAL(20, packets[1].size());
    TEST_ASSERT_EQUAL(20, packets[2].size());
    TEST_ASSERT_EQUAL(4,  packets[3].size());
}

static void test_emulate_qstarz_mode_byte_matches_fixtype(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);
    const uint8_t mode = static_cast<uint8_t>(packets[0][0]);
    TEST_ASSERT_EQUAL_UINT8(3, mode);
}

static void test_emulate_qstarz_mode_byte_1_when_no_fix(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 1;
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);
    const uint8_t mode = static_cast<uint8_t>(packets[0][0]);
    TEST_ASSERT_EQUAL_UINT8(1, mode);
}

static void test_emulate_qstarz_rcr_byte_is_T(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);
    const uint8_t rcr = static_cast<uint8_t>(packets[0][1]);
    TEST_ASSERT_EQUAL_UINT8('T', rcr);
}

void run_gps_tests(void)
{
    RUN_TEST(test_has_lock_false_when_invalid);
    RUN_TEST(test_has_lock_true_gps_3d);
    RUN_TEST(test_has_lock_true_gps_2d);
    RUN_TEST(test_has_lock_true_gps_fixtype_0);
    RUN_TEST(test_has_lock_false_gps_fixtype_1);
    RUN_TEST(test_has_lock_true_rtk_3d);
    RUN_TEST(test_has_lock_true_floatrtk_3d);
    RUN_TEST(test_has_3d_lock_true_gps_3d);
    RUN_TEST(test_has_3d_lock_false_gps_2d);
    RUN_TEST(test_has_3d_lock_false_fixtype_0);
    RUN_TEST(test_has_3d_lock_false_invalid);
    RUN_TEST(test_dop_100_gives_1_0);
    RUN_TEST(test_dop_150_gives_1_50);
    RUN_TEST(test_dop_200_gives_2_0);
    RUN_TEST(test_dop_250_gives_2_50);
    RUN_TEST(test_print_gps_geo_info_keys);
    RUN_TEST(test_print_gps_time_empty_without_3d_lock);
    RUN_TEST(test_print_gps_time_formats_unix_epoch);
    RUN_TEST(test_emulate_qstarz_packet_sizes);
    RUN_TEST(test_emulate_qstarz_mode_byte_matches_fixtype);
    RUN_TEST(test_emulate_qstarz_mode_byte_1_when_no_fix);
    RUN_TEST(test_emulate_qstarz_rcr_byte_is_T);
}
