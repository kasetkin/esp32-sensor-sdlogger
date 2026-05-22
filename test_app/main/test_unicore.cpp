#include "unity.h"
#include "unicore.h"
#include <utility>

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
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::SOL_COMPUTED),
                             std::to_underlying(parseSolutionStatus("SOL_COMPUTED", delay)));
    TEST_ASSERT_EQUAL_UINT16(0, delay);
}

static void test_parse_solution_status_sol_computed_with_prefix(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::SOL_COMPUTED),
                             std::to_underlying(parseSolutionStatus("17;SOL_COMPUTED", delay)));
    TEST_ASSERT_EQUAL_UINT16(17, delay);
}

static void test_parse_solution_status_no_convergence(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::NO_CONVERGENCE),
                             std::to_underlying(parseSolutionStatus("0;NO_CONVERGENCE", delay)));
}

static void test_parse_solution_status_cov_trace(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::COV_TRACE),
                             std::to_underlying(parseSolutionStatus("5;COV_TRACE", delay)));
}

static void test_parse_solution_status_unknown_returns_no_value(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::NO_VALUE),
                             std::to_underlying(parseSolutionStatus("0;UNKNOWN_STATUS", delay)));
}

// ── parsePositionType ─────────────────────────────────────────────────────────

static void test_parse_position_type_ppp(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PositionVelocityType::PPP),
                             std::to_underlying(parsePositionType("PPP")));
}

static void test_parse_position_type_ppp_lowercase(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PositionVelocityType::PPP),
                             std::to_underlying(parsePositionType("ppp")));
}

static void test_parse_position_type_single(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PositionVelocityType::SINGLE),
                             std::to_underlying(parsePositionType("SINGLE")));
}

static void test_parse_position_type_ppp_converging(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PositionVelocityType::PPP_CONVERGING),
                             std::to_underlying(parsePositionType("PPP_CONVERGING")));
}

static void test_parse_position_type_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PositionVelocityType::NO_VALUE),
                             std::to_underlying(parsePositionType("UNKNOWN_TYPE")));
}

// ── parseDatumId ──────────────────────────────────────────────────────────────

static void test_parse_datum_id_wgs84(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppDatumId::WGS84),
                             std::to_underlying(parseDatumId("WGS84")));
}

static void test_parse_datum_id_wgs84_lowercase(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppDatumId::WGS84),
                             std::to_underlying(parseDatumId("wgs84")));
}

static void test_parse_datum_id_b2b(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppDatumId::B2b),
                             std::to_underlying(parseDatumId("B2B")));
}

static void test_parse_datum_id_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppDatumId::NO_VALUE),
                             std::to_underlying(parseDatumId("UNKNOWN")));
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
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::GALILEO),
                             std::to_underlying(parsePppService(9901)));
}

static void test_parse_ppp_service_9959_beidou(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::BEIDOU),
                             std::to_underlying(parsePppService(9959)));
}

static void test_parse_ppp_service_9934_qzss(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::QZSS),
                             std::to_underlying(parsePppService(9934)));
}

static void test_parse_ppp_service_unknown_returns_no_value(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::NO_VALUE),
                             std::to_underlying(parsePppService(12345)));
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
        solutionStatusStr(PppSolutionStatus::SOL_COMPUTED).data());
}

static void test_position_type_str_ppp(void)
{
    TEST_ASSERT_EQUAL_STRING("PPP",
        positionTypeStr(PositionVelocityType::PPP).data());
}

static void test_service_id_str_galileo(void)
{
    TEST_ASSERT_EQUAL_STRING("GALILEO",
        serviceIdStr(PppService::GALILEO).data());
}

static void test_datum_id_str_wgs84(void)
{
    TEST_ASSERT_EQUAL_STRING("WGS84",
        datumIdStr(PppDatumId::WGS84).data());
}

static void test_datum_id_str_b2b(void)
{
    TEST_ASSERT_EQUAL_STRING("B2b",
        datumIdStr(PppDatumId::B2b).data());
}

// ── parseDegreesLatLon — negative values and precision edge cases ─────────────

static void test_parse_latlon_negative_south_pole(void)
{
    // -90.0 → -(90 * 10000000) = -900000000
    TEST_ASSERT_EQUAL_INT32(-900000000, parseDegreesLatLon("-90.0000000"));
}

static void test_parse_latlon_negative_dateline(void)
{
    // -180.0 → -(180 * 10000000) = -1800000000
    TEST_ASSERT_EQUAL_INT32(-1800000000, parseDegreesLatLon("-180.0000000"));
}

static void test_parse_latlon_negative_small(void)
{
    // -0.5 → -(0 * 10000000 + 5000000) = -5000000
    TEST_ASSERT_EQUAL_INT32(-5000000, parseDegreesLatLon("-0.5000000"));
}

static void test_parse_latlon_negative_invalid(void)
{
    // bare '-' followed by non-digit → still returns BAD
    TEST_ASSERT_EQUAL_INT32(PPP_BAD_LATLON, parseDegreesLatLon("-invalid"));
}

static void test_parse_latlon_180_degrees(void)
{
    TEST_ASSERT_EQUAL_INT32(1800000000, parseDegreesLatLon("180.0000000"));
}

static void test_parse_latlon_7_decimal_places(void)
{
    // all 7 decimal digits consumed: 1 * 10000000 + 1234567 = 11234567
    TEST_ASSERT_EQUAL_INT32(11234567, parseDegreesLatLon("1.1234567"));
}

static void test_parse_latlon_short_fraction(void)
{
    // only 2 decimal digits: 1 * 10000000 + 1200000 = 11200000
    TEST_ASSERT_EQUAL_INT32(11200000, parseDegreesLatLon("1.12"));
}

// ── parseSolutionStatus — INSUFFICIENT_OBS ────────────────────────────────────

static void test_parse_solution_status_insufficient_obs(void)
{
    uint16_t delay = 0;
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppSolutionStatus::INSUFFICIENT_OBS),
                             std::to_underlying(parseSolutionStatus("INSUFFICIENT_OBS", delay)));
}

// ── solutionStatusStr — completeness ─────────────────────────────────────────

static void test_solution_status_str_insufficient_obs(void)
{
    TEST_ASSERT_EQUAL_STRING("INSUFFICIENT_OBS",
        solutionStatusStr(PppSolutionStatus::INSUFFICIENT_OBS).data());
}

static void test_solution_status_str_no_value(void)
{
    TEST_ASSERT_EQUAL_STRING("NO_VALUE",
        solutionStatusStr(PppSolutionStatus::NO_VALUE).data());
}

// ── serviceIdStr — completeness ───────────────────────────────────────────────

static void test_service_id_str_rxn(void)
{
    TEST_ASSERT_EQUAL_STRING("RXN", serviceIdStr(PppService::RXN).data());
}

static void test_service_id_str_no_value(void)
{
    TEST_ASSERT_EQUAL_STRING("NO_VALUE", serviceIdStr(PppService::NO_VALUE).data());
}

// ── parseStationId — boundary / empty-string edge cases ──────────────────────

static void test_parse_station_id_empty_string(void)
{
    // starts_with returns false for empty string (no UB); atol("") == 0
    TEST_ASSERT_EQUAL_INT32(0, parseStationId(""));
}

static void test_parse_station_id_single_quote_char(void)
{
    // size == 1, size() > 2 is false → no strip; atol("\"") == 0
    TEST_ASSERT_EQUAL_INT32(0, parseStationId("\""));
}

static void test_parse_station_id_two_quotes_only(void)
{
    // size == 2, size() > 2 is false → no strip despite matching prefix+suffix
    TEST_ASSERT_EQUAL_INT32(0, parseStationId("\"\""));
}

// ── parsePppService — alternative station IDs ─────────────────────────────────

static void test_parse_ppp_service_9960_beidou(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::BEIDOU),
                             std::to_underlying(parsePppService(9960)));
}

static void test_parse_ppp_service_9936_qzss(void)
{
    TEST_ASSERT_EQUAL_UINT16(std::to_underlying(PppService::QZSS),
                             std::to_underlying(parsePppService(9936)));
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
    RUN_TEST(test_parse_station_id_empty_string);
    RUN_TEST(test_parse_station_id_single_quote_char);
    RUN_TEST(test_parse_station_id_two_quotes_only);
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
    RUN_TEST(test_parse_latlon_negative_south_pole);
    RUN_TEST(test_parse_latlon_negative_dateline);
    RUN_TEST(test_parse_latlon_negative_small);
    RUN_TEST(test_parse_latlon_negative_invalid);
    RUN_TEST(test_parse_latlon_180_degrees);
    RUN_TEST(test_parse_latlon_7_decimal_places);
    RUN_TEST(test_parse_latlon_short_fraction);
    RUN_TEST(test_parse_solution_status_insufficient_obs);
    RUN_TEST(test_solution_status_str_insufficient_obs);
    RUN_TEST(test_solution_status_str_no_value);
    RUN_TEST(test_service_id_str_rxn);
    RUN_TEST(test_service_id_str_no_value);
    RUN_TEST(test_parse_ppp_service_9960_beidou);
    RUN_TEST(test_parse_ppp_service_9936_qzss);
}
