#include "unity.h"
#include "unicore.h"

// ── parseDegreesLatLon ────────────────────────────────────────────────────────

static void test_parse_latlon_zero(void)
{
    TEST_ASSERT_EQUAL_INT32(0, parseDegreesLatLon("0.0000000"));
}

static void test_parse_latlon_positive(void)
{
    TEST_ASSERT_EQUAL_INT32(557558000, parseDegreesLatLon("55.7558000"));
}

static void test_parse_latlon_90_degrees(void)
{
    TEST_ASSERT_EQUAL_INT32(900000000, parseDegreesLatLon("90.0000000"));
}

static void test_parse_latlon_invalid_returns_bad(void)
{
    TEST_ASSERT_EQUAL_INT32(PPP_BAD_LATLON, parseDegreesLatLon("not_a_number"));
}

// ── parseSolutionStatus ───────────────────────────────────────────────────────

static void test_parse_solution_status_sol_computed_no_prefix(void)
{
    uint16_t delay = 999;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppSolutionStatus::SOL_COMPUTED),
                          static_cast<int>(parseSolutionStatus("SOL_COMPUTED", delay)));
    TEST_ASSERT_EQUAL_UINT16(0, delay);
}

static void test_parse_solution_status_sol_computed_with_prefix(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppSolutionStatus::SOL_COMPUTED),
                          static_cast<int>(parseSolutionStatus("17;SOL_COMPUTED", delay)));
    TEST_ASSERT_EQUAL_UINT16(17, delay);
}

static void test_parse_solution_status_no_convergence(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppSolutionStatus::NO_CONVERGENCE),
                          static_cast<int>(parseSolutionStatus("0;NO_CONVERGENCE", delay)));
}

static void test_parse_solution_status_cov_trace(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppSolutionStatus::COV_TRACE),
                          static_cast<int>(parseSolutionStatus("5;COV_TRACE", delay)));
}

static void test_parse_solution_status_unknown_returns_no_value(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppSolutionStatus::NO_VALUE),
                          static_cast<int>(parseSolutionStatus("0;UNKNOWN_STATUS", delay)));
}

// ── parsePositionType ─────────────────────────────────────────────────────────

static void test_parse_position_type_ppp(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionVelocityType::PPP),
                          static_cast<int>(parsePositionType("PPP")));
}

static void test_parse_position_type_ppp_lowercase(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionVelocityType::PPP),
                          static_cast<int>(parsePositionType("ppp")));
}

static void test_parse_position_type_single(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionVelocityType::SINGLE),
                          static_cast<int>(parsePositionType("SINGLE")));
}

static void test_parse_position_type_ppp_converging(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionVelocityType::PPP_CONVERGING),
                          static_cast<int>(parsePositionType("PPP_CONVERGING")));
}

static void test_parse_position_type_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionVelocityType::NO_VALUE),
                          static_cast<int>(parsePositionType("UNKNOWN_TYPE")));
}

// ── parseDatumId ──────────────────────────────────────────────────────────────

static void test_parse_datum_id_wgs84(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppDatumId::WGS84),
                          static_cast<int>(parseDatumId("WGS84")));
}

static void test_parse_datum_id_wgs84_lowercase(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppDatumId::WGS84),
                          static_cast<int>(parseDatumId("wgs84")));
}

static void test_parse_datum_id_b2b(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppDatumId::B2b),
                          static_cast<int>(parseDatumId("B2B")));
}

static void test_parse_datum_id_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppDatumId::NO_VALUE),
                          static_cast<int>(parseDatumId("UNKNOWN")));
}

// ── parseStationId ────────────────────────────────────────────────────────────

static void test_parse_station_id_strips_quotes(void)
{
    TEST_ASSERT_EQUAL_INT32(9901, parseStationId("\"9901\""));
}

static void test_parse_station_id_plain_number(void)
{
    TEST_ASSERT_EQUAL_INT32(9901, parseStationId("9901"));
}

static void test_parse_station_id_zero(void)
{
    TEST_ASSERT_EQUAL_INT32(0, parseStationId("0"));
}

// ── parsePppService ───────────────────────────────────────────────────────────

static void test_parse_ppp_service_9901_galileo(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppService::GALILEO),
                          static_cast<int>(parsePppService(9901)));
}

static void test_parse_ppp_service_9959_beidou(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppService::BEIDOU),
                          static_cast<int>(parsePppService(9959)));
}

static void test_parse_ppp_service_9934_qzss(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppService::QZSS),
                          static_cast<int>(parsePppService(9934)));
}

static void test_parse_ppp_service_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PppService::NO_VALUE),
                          static_cast<int>(parsePppService(12345)));
}

// ── computeUtxTime ────────────────────────────────────────────────────────────

static void test_compute_utx_time_gps_epoch(void)
{
    uint32_t outMs = 999;
    const uint32_t result = computeUtxTime(0, 0, 0, outMs);
    TEST_ASSERT_EQUAL_UINT32(315964800, result);
    TEST_ASSERT_EQUAL_UINT32(0, outMs);
}

static void test_compute_utx_time_one_week(void)
{
    uint32_t outMs = 0;
    const uint32_t week0 = computeUtxTime(0, 0, 0, outMs);
    const uint32_t week1 = computeUtxTime(1, 0, 0, outMs);
    TEST_ASSERT_EQUAL_UINT32(604800, week1 - week0);
}

static void test_compute_utx_time_milliseconds_split(void)
{
    uint32_t outMs = 0;
    const uint32_t result = computeUtxTime(0, 1500, 0, outMs);
    TEST_ASSERT_EQUAL_UINT32(500, outMs);
    TEST_ASSERT_EQUAL_UINT32(315964800 + 1, result);
}

static void test_compute_utx_time_leap_seconds(void)
{
    uint32_t outMs = 0;
    const uint32_t withLeap    = computeUtxTime(0, 0, 18, outMs);
    const uint32_t withoutLeap = computeUtxTime(0, 0,  0, outMs);
    TEST_ASSERT_EQUAL_UINT32(18, withoutLeap - withLeap);
}

// ── string conversion helpers ─────────────────────────────────────────────────

static void test_solution_status_str_sol_computed(void)
{
    TEST_ASSERT_EQUAL_STRING("SOL_COMPUTED",
        solutionStatusStr(PppSolutionStatus::SOL_COMPUTED).c_str());
}

static void test_position_type_str_ppp(void)
{
    TEST_ASSERT_EQUAL_STRING("PPP",
        positionTypeStr(PositionVelocityType::PPP).c_str());
}

static void test_service_id_str_galileo(void)
{
    TEST_ASSERT_EQUAL_STRING("GALILEO",
        serviceIdStr(PppService::GALILEO).c_str());
}

static void test_datum_id_str_wgs84(void)
{
    TEST_ASSERT_EQUAL_STRING("WGS84",
        datumIdStr(PppDatumId::WGS84).c_str());
}

static void test_datum_id_str_b2b(void)
{
    TEST_ASSERT_EQUAL_STRING("B2b",
        datumIdStr(PppDatumId::B2b).c_str());
}

void run_unicore_tests(void)
{
    RUN_TEST(test_parse_latlon_zero);
    RUN_TEST(test_parse_latlon_positive);
    RUN_TEST(test_parse_latlon_90_degrees);
    RUN_TEST(test_parse_latlon_invalid_returns_bad);
    RUN_TEST(test_parse_solution_status_sol_computed_no_prefix);
    RUN_TEST(test_parse_solution_status_sol_computed_with_prefix);
    RUN_TEST(test_parse_solution_status_no_convergence);
    RUN_TEST(test_parse_solution_status_cov_trace);
    RUN_TEST(test_parse_solution_status_unknown_returns_no_value);
    RUN_TEST(test_parse_position_type_ppp);
    RUN_TEST(test_parse_position_type_ppp_lowercase);
    RUN_TEST(test_parse_position_type_single);
    RUN_TEST(test_parse_position_type_ppp_converging);
    RUN_TEST(test_parse_position_type_unknown_returns_no_value);
    RUN_TEST(test_parse_datum_id_wgs84);
    RUN_TEST(test_parse_datum_id_wgs84_lowercase);
    RUN_TEST(test_parse_datum_id_b2b);
    RUN_TEST(test_parse_datum_id_unknown_returns_no_value);
    RUN_TEST(test_parse_station_id_strips_quotes);
    RUN_TEST(test_parse_station_id_plain_number);
    RUN_TEST(test_parse_station_id_zero);
    RUN_TEST(test_parse_ppp_service_9901_galileo);
    RUN_TEST(test_parse_ppp_service_9959_beidou);
    RUN_TEST(test_parse_ppp_service_9934_qzss);
    RUN_TEST(test_parse_ppp_service_unknown_returns_no_value);
    RUN_TEST(test_compute_utx_time_gps_epoch);
    RUN_TEST(test_compute_utx_time_one_week);
    RUN_TEST(test_compute_utx_time_milliseconds_split);
    RUN_TEST(test_compute_utx_time_leap_seconds);
    RUN_TEST(test_solution_status_str_sol_computed);
    RUN_TEST(test_position_type_str_ppp);
    RUN_TEST(test_service_id_str_galileo);
    RUN_TEST(test_datum_id_str_wgs84);
    RUN_TEST(test_datum_id_str_b2b);
}
