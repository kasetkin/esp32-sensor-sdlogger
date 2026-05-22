#include "unity.h"
#include <cstring>
#include "gpstask.h"
#include "unicore.h"

// ── helpers ───────────────────────────────────────────────────────────────────

static PppInfo makePppInfo(void)
{
    PppInfo p;
    p.utxSeconds     = 1700000000;
    p.millisecs      = 0;
    p.solutionStatus = PppSolutionStatus::SOL_COMPUTED;
    p.positionType   = PositionVelocityType::PPP;
    p.serviceId      = PppService::GALILEO;
    p.datumId        = PppDatumId::WGS84;
    p.solutionAge    = 10;
    p.lat            = 557558000;   // 55.7558 deg (× 1e-7)
    p.lon            = 376173000;   // 37.6173 deg (× 1e-7)
    p.alt            = 150;
    p.latStdDev      = 0.1f;
    p.lonStdDev      = 0.1f;
    p.altStdDev      = 0.2f;
    p.satellites     = 12;
    p.stationId      = 7;
    return p;
}

// ── GpsTask::printPppTimeInfo ─────────────────────────────────────────────────

static void test_ppp_time_all_keys_present(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains("PPP_SOLUTION_STATUS;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_POSITION;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_SERVICE;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_DATUM;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_DT;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_TIME;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_AGE;"));
}

static void test_ppp_time_enum_strings(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains("PPP_SOLUTION_STATUS;SOL_COMPUTED"));
    TEST_ASSERT_TRUE(result.contains(";PPP_POSITION;PPP;"));
    TEST_ASSERT_TRUE(result.contains(";PPP_SERVICE;GALILEO"));
    TEST_ASSERT_TRUE(result.contains(";PPP_DATUM;WGS84"));
}

static void test_ppp_time_unix_epoch(void)
{
    PppInfo p = makePppInfo();
    p.utxSeconds = 0;
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains(";PPP_DT;1970-01-01T00:00:00Z"));
    TEST_ASSERT_TRUE(result.contains(";PPP_TIME;0"));
}

static void test_ppp_time_gps_epoch(void)
{
    PppInfo p = makePppInfo();
    p.utxSeconds = 315964800;  // 1980-01-06T00:00:00Z
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains(";PPP_DT;1980-01-06T00:00:00Z"));
    TEST_ASSERT_TRUE(result.contains(";PPP_TIME;315964800"));
}

static void test_ppp_time_no_millisecs_when_zero(void)
{
    PppInfo p = makePppInfo();
    p.utxSeconds = 0;
    p.millisecs  = 0;
    const std::string result = GpsTask::printPppTimeInfo(p);

    // no dot inserted when millisecs == 0
    TEST_ASSERT_TRUE(result.contains("T00:00:00Z"));
}

static void test_ppp_time_millisecs_500(void)
{
    PppInfo p = makePppInfo();
    p.utxSeconds = 0;
    p.millisecs  = 500;
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains("T00:00:00.500Z"));
}

static void test_ppp_time_millisecs_zero_padded(void)
{
    PppInfo p = makePppInfo();
    p.utxSeconds = 0;
    p.millisecs  = 50;   // toStringWithZeros(50, 3) → "050", not "50"
    const std::string result = GpsTask::printPppTimeInfo(p);

    TEST_ASSERT_TRUE(result.contains("T00:00:00.050Z"));
}

// ── GpsTask::printPppGeoInfo ──────────────────────────────────────────────────

static void test_ppp_geo_all_keys_present(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_TRUE(result.contains("PPP_LAT;"));
    TEST_ASSERT_TRUE(result.contains("PPP_LON;"));
    TEST_ASSERT_TRUE(result.contains("PPP_GNSS_OFFSET;"));
    TEST_ASSERT_TRUE(result.contains("PPP_ALT;"));
    TEST_ASSERT_TRUE(result.contains("PPP_SATS;"));
    TEST_ASSERT_TRUE(result.contains("PPP_STATION_ID;"));
    TEST_ASSERT_TRUE(result.contains("PPP_LATSTDDEV;"));
    TEST_ASSERT_TRUE(result.contains("PPP_LONSTDDEV;"));
    TEST_ASSERT_TRUE(result.contains("PPP_ALTSTDDEV;"));
}

static void test_ppp_geo_starts_ends_semicolon(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_FALSE(result.empty());
    TEST_ASSERT_EQUAL_UINT8(';', static_cast<uint8_t>(result.front()));
    TEST_ASSERT_EQUAL_UINT8(';', static_cast<uint8_t>(result.back()));
}

static void test_ppp_geo_lat_lon_conversion(void)
{
    PppInfo p = makePppInfo();
    p.lat = 100000000;   // 10.0 degrees
    p.lon = 200000000;   // 20.0 degrees
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_TRUE(result.contains("PPP_LAT;10"));
    TEST_ASSERT_TRUE(result.contains("PPP_LON;20"));
}

static void test_ppp_geo_negative_coords(void)
{
    PppInfo p = makePppInfo();
    p.lat = -900000000;   // -90.0 deg (South Pole)
    p.lon = -1800000000;  // -180.0 deg (Date Line west)
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_TRUE(result.contains("PPP_LAT;-"));
    TEST_ASSERT_TRUE(result.contains("PPP_LON;-"));
}

static void test_ppp_geo_zero_gnss_offset(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_TRUE(result.contains("PPP_GNSS_OFFSET;0"));
}

static void test_ppp_geo_nonzero_gnss_offset(void)
{
    const PppInfo p = makePppInfo();
    const std::string result = GpsTask::printPppGeoInfo(p, 25.5);

    TEST_ASSERT_TRUE(result.contains("PPP_GNSS_OFFSET;25"));
}

static void test_ppp_geo_integer_fields(void)
{
    PppInfo p = makePppInfo();
    p.alt        = 12345;
    p.satellites = 9;
    p.stationId  = 42;
    const std::string result = GpsTask::printPppGeoInfo(p, 0.0);

    TEST_ASSERT_TRUE(result.contains("PPP_ALT;12345"));
    TEST_ASSERT_TRUE(result.contains("PPP_SATS;9"));
    TEST_ASSERT_TRUE(result.contains("PPP_STATION_ID;42"));
}

void run_ppp_tests(void)
{
    RUN_TEST(test_ppp_time_all_keys_present);
    RUN_TEST(test_ppp_time_enum_strings);
    RUN_TEST(test_ppp_time_unix_epoch);
    RUN_TEST(test_ppp_time_gps_epoch);
    RUN_TEST(test_ppp_time_no_millisecs_when_zero);
    RUN_TEST(test_ppp_time_millisecs_500);
    RUN_TEST(test_ppp_time_millisecs_zero_padded);
    RUN_TEST(test_ppp_geo_all_keys_present);
    RUN_TEST(test_ppp_geo_starts_ends_semicolon);
    RUN_TEST(test_ppp_geo_lat_lon_conversion);
    RUN_TEST(test_ppp_geo_negative_coords);
    RUN_TEST(test_ppp_geo_zero_gnss_offset);
    RUN_TEST(test_ppp_geo_nonzero_gnss_offset);
    RUN_TEST(test_ppp_geo_integer_fields);
}
