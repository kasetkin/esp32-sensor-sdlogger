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

static void test_dop_100_gives_1_00(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 100);
    TEST_ASSERT_EQUAL_STRING("1.00", r.c_str());
}

static void test_dop_150_gives_1_50(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 150);
    TEST_ASSERT_EQUAL_STRING("1.50", r.c_str());
}

static void test_dop_200_gives_2_00(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 200);
    TEST_ASSERT_EQUAL_STRING("2.00", r.c_str());
}

static void test_dop_250_gives_2_50(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 250);
    TEST_ASSERT_EQUAL_STRING("2.50", r.c_str());
}

static void test_dop_0_gives_0_00(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 0);
    TEST_ASSERT_EQUAL_STRING("0.00", r.c_str());
}

static void test_dop_101_gives_1_01(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 101);
    TEST_ASSERT_EQUAL_STRING("1.01", r.c_str());
}

static void test_dop_105_gives_1_05(void)
{
    // Previously produced "1.5" — rem=5 was not zero-padded to "05"
    std::string r;
    GpsTask::dopToMeters(r, 105);
    TEST_ASSERT_EQUAL_STRING("1.05", r.c_str());
}

static void test_dop_1000_gives_10_00(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 1000);
    TEST_ASSERT_EQUAL_STRING("10.00", r.c_str());
}

static void test_dop_5000_gives_50_00(void)
{
    std::string r;
    GpsTask::dopToMeters(r, 5000);
    TEST_ASSERT_EQUAL_STRING("50.00", r.c_str());
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

    TEST_ASSERT_TRUE(result.find("LAT;1")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("LON;2")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("ALT;10")  != std::string::npos);
    TEST_ASSERT_TRUE(result.find("UNDUL;5") != std::string::npos);
    TEST_ASSERT_TRUE(result.find("PDOP;2.00")      != std::string::npos);
    TEST_ASSERT_TRUE(result.find("HDOP;1.50")      != std::string::npos);
    TEST_ASSERT_TRUE(result.find("VDOP;2.50")      != std::string::npos);
}

static void test_print_gps_geo_info_zero_coords(void)
{
    GpsInfo info;
    info.lat      = 0.0;
    info.lon      = 0.0;
    info.altitude = 0.0;
    info.geoidAlt = 0.0;
    info.gsaPDOP  = 100;
    info.gsaHDOP  = 100;
    info.gsaVDOP  = 100;

    const std::string result = GpsTask::printGpsGeoInfo(info);

    TEST_ASSERT_TRUE(result.find("LAT;0")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("LON;0")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("ALT;0")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("UNDUL;0") != std::string::npos);
}

static void test_print_gps_geo_info_negative_coords(void)
{
    GpsInfo info;
    info.lat      = -90.0;    // South Pole
    info.lon      = -180.0;   // Date Line west
    info.altitude = -430.0;   // Dead Sea depth
    info.geoidAlt = -110.0;   // Indian Ocean geoid low
    info.gsaPDOP  = 200;
    info.gsaHDOP  = 150;
    info.gsaVDOP  = 250;

    const std::string result = GpsTask::printGpsGeoInfo(info);

    TEST_ASSERT_TRUE(result.find("LAT;-")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("LON;-")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("ALT;-")   != std::string::npos);
    TEST_ASSERT_TRUE(result.find("UNDUL;-") != std::string::npos);
}

static void test_print_gps_geo_info_max_altitude(void)
{
    GpsInfo info;
    info.lat      = 27.988;
    info.lon      = 86.925;
    info.altitude = 8850.0;   // Everest summit
    info.geoidAlt = -28.0;    // geoid separation near Everest
    info.gsaPDOP  = 200;
    info.gsaHDOP  = 150;
    info.gsaVDOP  = 250;

    const std::string result = GpsTask::printGpsGeoInfo(info);

    TEST_ASSERT_TRUE(result.find("ALT;8850") != std::string::npos);
    TEST_ASSERT_TRUE(result.find("UNDUL;-")  != std::string::npos);
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

static void test_print_gps_time_gps_epoch(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.worldTime = std::chrono::system_clock::from_time_t(315964800); // 1980-01-06

    const std::string result = GpsTask::printGpsTimeInfo(info);
    TEST_ASSERT_TRUE(result.find("DT;1980-01-06T00:00:00Z") != std::string::npos);
    TEST_ASSERT_TRUE(result.find("GNSSSEC;315964800")       != std::string::npos);
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

static void test_emulate_qstarz_equator_prime_meridian(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.lat       = 0.0;
    info.lon       = 0.0;
    info.altitude  = 0.0;
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);
    TEST_ASSERT_EQUAL(4,  packets.size());
    TEST_ASSERT_EQUAL(20, packets[0].size());
    TEST_ASSERT_EQUAL(20, packets[1].size());
    TEST_ASSERT_EQUAL(20, packets[2].size());
    TEST_ASSERT_EQUAL(4,  packets[3].size());
}

// ── GpsInfo::satellites (std::flat_set<SatelliteInfo>) ───────────────────────

static void test_satellite_set_deduplicates_same_prn(void)
{
    GpsInfo info;
    info.satellites.insert(SatelliteInfo{.prn = 5});
    info.satellites.insert(SatelliteInfo{.prn = 5}); // duplicate
    TEST_ASSERT_EQUAL(1, info.satellites.size());
}

static void test_satellite_set_is_sorted(void)
{
    GpsInfo info;
    info.satellites.insert(SatelliteInfo{.prn = 20});
    info.satellites.insert(SatelliteInfo{.prn = 3});
    info.satellites.insert(SatelliteInfo{.prn = 10});
    auto it = info.satellites.begin();
    TEST_ASSERT_EQUAL_UINT8(3,  it->prn); ++it;
    TEST_ASSERT_EQUAL_UINT8(10, it->prn); ++it;
    TEST_ASSERT_EQUAL_UINT8(20, it->prn);
}

static void test_satellite_set_counts_correctly(void)
{
    GpsInfo info;
    for (uint8_t prn = 1; prn <= 12; ++prn)
        info.satellites.insert(SatelliteInfo{.prn = prn});
    TEST_ASSERT_EQUAL(12, info.satellites.size());
}

static void test_satellite_set_move_clears_source(void)
{
    GpsInfo src;
    src.satellites.insert(SatelliteInfo{.prn = 7});
    GpsInfo dst;
    dst.satellites = std::move(src.satellites);
    TEST_ASSERT_EQUAL(0, src.satellites.size());
    TEST_ASSERT_EQUAL(1, dst.satellites.size());
}

static void test_print_gps_geo_info_satellite_count(void)
{
    GpsInfo info;
    info.lat      = 0.0;
    info.lon      = 0.0;
    info.altitude = 0.0;
    info.geoidAlt = 0.0;
    info.gsaPDOP  = 100;
    info.gsaHDOP  = 100;
    info.gsaVDOP  = 100;
    info.satellites.insert(SatelliteInfo{.prn = 1});
    info.satellites.insert(SatelliteInfo{.prn = 2});
    info.satellites.insert(SatelliteInfo{.prn = 3});
    const std::string result = GpsTask::printGpsGeoInfo(info);
    TEST_ASSERT_TRUE(result.find("SATS;3") != std::string::npos);
}

// ── GpsTask::hasLock / has3DLock — untested Quality values ───────────────────

static void test_has_lock_true_dgps_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::DGPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_true_pps_3d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::PPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::hasLock(info));
}

static void test_has_lock_false_estimated(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::Estimated;
    info.fixType = 3;
    TEST_ASSERT_FALSE(GpsTask::hasLock(info));
}

static void test_has_3d_lock_true_dgps(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::DGPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::has3DLock(info));
}

static void test_has_3d_lock_true_pps(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::PPS;
    info.fixType = 3;
    TEST_ASSERT_TRUE(GpsTask::has3DLock(info));
}

static void test_has_3d_lock_false_pps_2d(void)
{
    GpsInfo info;
    info.quality = TinyGPSLocation::Quality::PPS;
    info.fixType = 2;
    TEST_ASSERT_FALSE(GpsTask::has3DLock(info));
}

// ── GpsTask::dopToMeters — sentinel value ────────────────────────────────────

static void test_dop_bad_value(void)
{
    // BAD_DOP=666000000: div(666000000,100) → quot=6660000, rem=0
    std::string r;
    GpsTask::dopToMeters(r, GpsInfo::BAD_DOP);
    TEST_ASSERT_EQUAL_STRING("6660000.00", r.c_str());
}

static void test_emulate_qstarz_south_pole_dateline(void)
{
    GpsInfo info;
    info.quality   = TinyGPSLocation::Quality::GPS;
    info.fixType   = 3;
    info.lat       = -90.0;   // South Pole
    info.lon       = -180.0;  // Date Line west
    info.altitude  = -430.0;  // Dead Sea depth
    info.worldTime = std::chrono::system_clock::from_time_t(1700000000);

    const auto packets = GpsTask::emulateQstarzBinary(info);
    TEST_ASSERT_EQUAL(4,  packets.size());
    TEST_ASSERT_EQUAL(20, packets[0].size());
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
    RUN_TEST(test_dop_100_gives_1_00);
    RUN_TEST(test_dop_150_gives_1_50);
    RUN_TEST(test_dop_200_gives_2_00);
    RUN_TEST(test_dop_250_gives_2_50);
    RUN_TEST(test_dop_0_gives_0_00);
    RUN_TEST(test_dop_101_gives_1_01);
    RUN_TEST(test_dop_105_gives_1_05);
    RUN_TEST(test_dop_1000_gives_10_00);
    RUN_TEST(test_dop_5000_gives_50_00);
    RUN_TEST(test_print_gps_geo_info_keys);
    RUN_TEST(test_print_gps_geo_info_zero_coords);
    RUN_TEST(test_print_gps_geo_info_negative_coords);
    RUN_TEST(test_print_gps_geo_info_max_altitude);
    RUN_TEST(test_print_gps_time_empty_without_3d_lock);
    RUN_TEST(test_print_gps_time_formats_unix_epoch);
    RUN_TEST(test_print_gps_time_gps_epoch);
    RUN_TEST(test_emulate_qstarz_packet_sizes);
    RUN_TEST(test_emulate_qstarz_mode_byte_matches_fixtype);
    RUN_TEST(test_emulate_qstarz_mode_byte_1_when_no_fix);
    RUN_TEST(test_emulate_qstarz_rcr_byte_is_T);
    RUN_TEST(test_emulate_qstarz_equator_prime_meridian);
    RUN_TEST(test_emulate_qstarz_south_pole_dateline);
    RUN_TEST(test_has_lock_true_dgps_3d);
    RUN_TEST(test_has_lock_true_pps_3d);
    RUN_TEST(test_has_lock_false_estimated);
    RUN_TEST(test_has_3d_lock_true_dgps);
    RUN_TEST(test_has_3d_lock_true_pps);
    RUN_TEST(test_has_3d_lock_false_pps_2d);
    RUN_TEST(test_dop_bad_value);
    RUN_TEST(test_satellite_set_deduplicates_same_prn);
    RUN_TEST(test_satellite_set_is_sorted);
    RUN_TEST(test_satellite_set_counts_correctly);
    RUN_TEST(test_satellite_set_move_clears_source);
    RUN_TEST(test_print_gps_geo_info_satellite_count);
}
