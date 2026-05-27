#include "unity.h"
#include "TinyGPSPlus.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <string_view>

// Feed an NMEA sentence into the parser one character at a time, with a
// correct checksum auto-computed from the body. `body` is everything between
// '$' and '*' (exclusive), without leading '$'.
static void feedSentence(TinyGPSPlus& gps, std::string_view body)
{
    gps.encode('$');
    uint8_t chk = 0;
    for (const char c : body) {
        gps.encode(c);
        chk ^= static_cast<uint8_t>(c);
    }
    gps.encode('*');
    std::array<char, 3> hex{};
    std::snprintf(hex.data(), hex.size(), "%02X", chk);
    gps.encode(hex[0]);
    gps.encode(hex[1]);
    gps.encode('\r');
    gps.encode('\n');
}

// Same but injects a deliberately wrong checksum, so the parser rejects the sentence.
static void feedSentenceBadChecksum(TinyGPSPlus& gps, std::string_view body)
{
    gps.encode('$');
    for (const char c : body) {
        gps.encode(c);
    }
    gps.encode('*');
    gps.encode('0');
    gps.encode('0');
    gps.encode('\r');
    gps.encode('\n');
}

// ── Single GPS GSA (NMEA 4.10) ──────────────────────────────────────────────

static void test_satellites_single_gpgsa_full(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GPGSA,A,3,3,4,7,8,15,17,19,22,24,29,31,32,1.0,1.2,1.5,1");

    TEST_ASSERT_TRUE(gps.satellites.isUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_FALSE(gps.satellites.isUpdated()); // consume cleared it

    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixMode::Auto), static_cast<int>(data->mode));
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix3D), static_cast<int>(data->fixType));
    TEST_ASSERT_EQUAL_INT32(100, data->pdop);
    TEST_ASSERT_EQUAL_INT32(120, data->hdop);
    TEST_ASSERT_EQUAL_INT32(150, data->vdop);

    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(12u, sats.size());
    TEST_ASSERT_EQUAL_UINT8(3, sats.front().prn);
    TEST_ASSERT_EQUAL_UINT8(32, sats.back().prn);
    for (const auto& s : sats) {
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::GPS), static_cast<int>(s.systemId));
        TEST_ASSERT_TRUE(s.inSolution);
    }
}

// ── GSA with empty PRN slots ────────────────────────────────────────────────

static void test_satellites_gsa_partial_prns(void)
{
    TinyGPSPlus gps;
    // Only 3 PRNs filled; remaining 9 PRN slots empty
    feedSentence(gps, "GPGSA,A,2,3,4,7,,,,,,,,,,1.0,1.2,1.5,1");

    TEST_ASSERT_TRUE(gps.satellites.isUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix2D), static_cast<int>(data->fixType));

    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(3u, sats.size());
    TEST_ASSERT_EQUAL_UINT8(3, sats[0].prn);
    TEST_ASSERT_EQUAL_UINT8(4, sats[1].prn);
    TEST_ASSERT_EQUAL_UINT8(7, sats[2].prn);
}

// ── Legacy GSA without System ID field (pre-NMEA 4.10) ──────────────────────

static void test_satellites_gsa_legacy_no_system_id(void)
{
    TinyGPSPlus gps;
    // 17 fields total (mode, fix, 12 PRNs, PDOP, HDOP, VDOP) — no System ID
    feedSentence(gps, "GPGSA,A,3,3,4,7,8,15,17,19,22,24,29,31,32,1.0,1.2,1.5");

    TEST_ASSERT_TRUE(gps.satellites.isUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(12u, sats.size());
    // Talker prefix "GP" should map to GPS even without the System ID field
    for (const auto& s : sats) {
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::GPS), static_cast<int>(s.systemId));
    }
}

// ── Multi-constellation accumulation across consecutive GN GSA sentences ────

static void test_satellites_multi_constellation_accumulation(void)
{
    TinyGPSPlus gps;

    // GPS (System ID 1): PRNs 3,4,7,8,15
    feedSentence(gps, "GNGSA,A,3,3,4,7,8,15,,,,,,,,1.1,1.2,1.3,1");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(5u, data->inSolutionCount);
    }

    // GLONASS (System ID 2): PRNs 65,66,67
    feedSentence(gps, "GNGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3,2");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(8u, data->inSolutionCount);
    }

    // Galileo (System ID 3): PRNs 5,9
    feedSentence(gps, "GNGSA,A,3,5,9,,,,,,,,,,,1.1,1.2,1.3,3");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(10u, sats.size());

    // Each satellite carries its constellation; spot-check by PRN range.
    int gpsCount = 0;
    int gloCount = 0;
    int galCount = 0;
    for (const auto& s : sats) {
        if (s.systemId == GnssSystemId::GPS)
            ++gpsCount;
        else if (s.systemId == GnssSystemId::GLONASS)
            ++gloCount;
        else if (s.systemId == GnssSystemId::Galileo)
            ++galCount;
    }
    TEST_ASSERT_EQUAL(5, gpsCount);
    TEST_ASSERT_EQUAL(3, gloCount);
    TEST_ASSERT_EQUAL(2, galCount);
}

// ── Per-system replacement (a new GSA for system X replaces only X) ─────────

static void test_satellites_per_system_replacement(void)
{
    TinyGPSPlus gps;

    // Initial GPS + GLONASS snapshots
    feedSentence(gps, "GNGSA,A,3,3,4,7,,,,,,,,,,1.1,1.2,1.3,1");
    feedSentence(gps, "GNGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3,2");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(6u, data->inSolutionCount);
    }

    // New GPS sentence with a different PRN list — replaces only GPS slot
    feedSentence(gps, "GNGSA,A,3,10,11,12,13,,,,,,,,,1.1,1.2,1.3,1");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->satellites();
    // 4 (new GPS) + 3 (unchanged GLONASS) = 7
    TEST_ASSERT_EQUAL(7u, sats.size());

    bool sawOldGpsPrn3 = false;
    bool sawNewGpsPrn10 = false;
    bool sawGlonassPrn65 = false;
    for (const auto& s : sats) {
        if (s.systemId == GnssSystemId::GPS && s.prn == 3)
            sawOldGpsPrn3 = true;
        if (s.systemId == GnssSystemId::GPS && s.prn == 10)
            sawNewGpsPrn10 = true;
        if (s.systemId == GnssSystemId::GLONASS && s.prn == 65)
            sawGlonassPrn65 = true;
    }
    TEST_ASSERT_FALSE(sawOldGpsPrn3);   // old GPS list dropped
    TEST_ASSERT_TRUE(sawNewGpsPrn10);   // new GPS list present
    TEST_ASSERT_TRUE(sawGlonassPrn65);  // GLONASS untouched
}

// ── Talker-derived SystemID (GLGSA, no System ID field) ─────────────────────

static void test_satellites_talker_derived_system_id(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GLGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(3u, sats.size());
    for (const auto& s : sats) {
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::GLONASS), static_cast<int>(s.systemId));
    }
}

// ── Bad checksum: no commit, accumulator unchanged ──────────────────────────

static void test_satellites_bad_checksum_no_commit(void)
{
    TinyGPSPlus gps;

    // Prime with a good sentence first so we have known committed state
    feedSentence(gps, "GPGSA,A,3,3,4,7,,,,,,,,,,2.0,2.0,2.0,1");
    TEST_ASSERT_TRUE(gps.satellites.isUpdated());
    {
        const auto primed = gps.satellites.consume(); // clears isUpdated
        TEST_ASSERT_TRUE(primed.has_value());
        TEST_ASSERT_EQUAL(3u, primed->inSolutionCount);
    }
    TEST_ASSERT_FALSE(gps.satellites.isUpdated());

    // Now feed a sentence with deliberately wrong checksum
    feedSentenceBadChecksum(gps, "GPGSA,A,3,9,10,11,12,13,,,,,,,,1.0,1.0,1.0,1");

    TEST_ASSERT_FALSE(gps.satellites.isUpdated());   // commit never fired
    TEST_ASSERT_FALSE(gps.satellites.consume().has_value()); // nothing to consume
}

// ── GGA renamed `satellitesUsedCount` smoke test ────────────────────────────

static void test_satellites_used_count_from_gga(void)
{
    TinyGPSPlus gps;
    // Minimal valid GGA with satellites-used = 8
    feedSentence(gps, "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");

    TEST_ASSERT_TRUE(gps.satellitesUsedCount.isUpdated());
    const auto data = gps.satellitesUsedCount.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_EQUAL_UINT32(8, data->raw);
}

// ── Real UM980 capture: one full epoch from a production stream ─────────────
// Sentences derived from docs/2026-04-24-sdlogger_UM980_nmea.csv,
// epoch t = 000001.00 UTC. The receiver emits five GN GSA sentences
// (System IDs 1..5 = GPS, GLONASS, Galileo, BeiDou, QZSS) followed by RMC
// and GGA. The PPPNAVA #-protocol sentence is omitted because its 32-bit CRC
// doesn't fit the simple NMEA feedSentence helper and isn't needed here.
//
// Lat/lon/altitude/geoid have been replaced with a synthetic mid-Pacific
// coordinate (~25.5°N 130.8°E, in the Philippine Sea) to scrub the original
// recording location. Format and precision (8 fractional digits in minutes)
// match real UM980 output so the tests still exercise the same parser paths.

static constexpr std::array<std::string_view, 5> kUm980EpochGsas = {
    "GNGSA,M,3,07,09,17,30,01,22,,,,,,,1.2,0.8,1.0,1",
    "GNGSA,M,3,71,83,84,,,,,,,,,,1.2,0.8,1.0,2",
    "GNGSA,M,3,07,08,30,27,,,,,,,,,1.2,0.8,1.0,3",
    "GNGSA,M,3,01,02,03,38,41,08,32,10,34,13,07,25,1.2,0.8,1.0,4",
    "GNGSA,M,3,07,03,,,,,,,,,,,1.2,0.8,1.0,5",
};
static constexpr std::string_view kUm980EpochRmc =
    "GNRMC,000001.00,A,2530.12345678,N,13045.87654321,E,0.008,204.6,240426,3.6,E,P,S";
static constexpr std::string_view kUm980EpochGga =
    "GNGGA,000001.00,2530.12345678,N,13045.87654321,E,5,27,0.8,12.3400,M,-28.5000,M,9.0,9901";

static void test_satellites_real_um980_5_constellations(void)
{
    TinyGPSPlus gps;
    for (const auto sentence : kUm980EpochGsas)
        feedSentence(gps, sentence);

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    // Counts per constellation from the source data:
    //  GPS     : 6  (07,09,17,30,01,22)
    //  GLONASS : 3  (71,83,84)
    //  Galileo : 4  (07,08,30,27)
    //  BeiDou  : 12 (01,02,03,38,41,08,32,10,34,13,07,25)
    //  QZSS    : 2  (07,03)
    //  Total   : 27 — also reported as the satellites-used field of GGA
    const auto sats = data->satellites();
    TEST_ASSERT_EQUAL(27u, sats.size());

    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixMode::Manual), static_cast<int>(data->mode));
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix3D),  static_cast<int>(data->fixType));
    TEST_ASSERT_EQUAL_INT32(120, data->pdop);
    TEST_ASSERT_EQUAL_INT32(80,  data->hdop);
    TEST_ASSERT_EQUAL_INT32(100, data->vdop);

    int gpsCount = 0;
    int gloCount = 0;
    int galCount = 0;
    int bdsCount = 0;
    int qzsCount = 0;
    for (const auto& s : sats) {
        switch (s.systemId) {
            case GnssSystemId::GPS:     ++gpsCount; break;
            case GnssSystemId::GLONASS: ++gloCount; break;
            case GnssSystemId::Galileo: ++galCount; break;
            case GnssSystemId::BeiDou:  ++bdsCount; break;
            case GnssSystemId::QZSS:    ++qzsCount; break;
            case GnssSystemId::Unknown: break;
        }
    }
    TEST_ASSERT_EQUAL(6,  gpsCount);
    TEST_ASSERT_EQUAL(3,  gloCount);
    TEST_ASSERT_EQUAL(4,  galCount);
    TEST_ASSERT_EQUAL(12, bdsCount);
    TEST_ASSERT_EQUAL(2,  qzsCount);

    // Same PRN appears in multiple constellations — verify each lands in
    // the correct system rather than aliasing.
    auto contains = [&](uint8_t prn, GnssSystemId sys) {
        for (const auto& s : sats)
            if (s.prn == prn && s.systemId == sys)
                return true;
        return false;
    };
    TEST_ASSERT_TRUE(contains(7,  GnssSystemId::GPS));
    TEST_ASSERT_TRUE(contains(71, GnssSystemId::GLONASS));
    TEST_ASSERT_TRUE(contains(7,  GnssSystemId::Galileo));
    TEST_ASSERT_TRUE(contains(8,  GnssSystemId::Galileo));
    TEST_ASSERT_TRUE(contains(38, GnssSystemId::BeiDou));
    TEST_ASSERT_TRUE(contains(7,  GnssSystemId::QZSS));
    TEST_ASSERT_TRUE(contains(3,  GnssSystemId::QZSS));
}

static void test_satellites_real_um980_full_epoch_with_gga_rmc(void)
{
    TinyGPSPlus gps;
    for (const auto sentence : kUm980EpochGsas)
        feedSentence(gps, sentence);
    feedSentence(gps, kUm980EpochRmc);
    feedSentence(gps, kUm980EpochGga);

    // GGA's satellites-used field matches the GSA union (sanity cross-check)
    const auto sats = gps.satellites.consume();
    const auto used = gps.satellitesUsedCount.consume();
    TEST_ASSERT_TRUE(sats.has_value());
    TEST_ASSERT_TRUE(used.has_value());
    TEST_ASSERT_EQUAL_UINT32(27, used->raw);
    TEST_ASSERT_EQUAL(27u, sats->inSolutionCount);

    // Synthetic position: 25°30.12345678'N = 25.502057..°,  130°45.87654321'E = 130.764609..°
    // The 8th fractional digit is truncated by parseDegrees (capped at 7).
    const auto loc = gps.location.consume();
    TEST_ASSERT_TRUE(loc.has_value());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 25.50205761, loc->latDeg());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 130.76460905, loc->lngDeg());

    // Fix quality from GGA term 6 = '5' (FloatRTK)
    TEST_ASSERT_EQUAL(static_cast<int>(TinyGPSLocation::Quality::FloatRTK),
                      static_cast<int>(loc->fixQuality));

    // Altitude and geoid height — TinyGPSAltitude stores int32 × 100, so
    // expect 2-decimal precision (truncates 12.3400 → 12.34).
    const auto alt = gps.altitude.consume();
    const auto geo = gps.geoidHeight.consume();
    TEST_ASSERT_TRUE(alt.has_value());
    TEST_ASSERT_TRUE(geo.has_value());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 12.34, alt->meters());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, -28.50, geo->meters());

    // GGA term 8 HDOP = 0.8
    const auto hdop = gps.hdop.consume();
    TEST_ASSERT_TRUE(hdop.has_value());
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.8, hdop->hdop());

    // Date from RMC: 240426 → 24 April 2026
    const auto date = gps.date.consume();
    TEST_ASSERT_TRUE(date.has_value());
    TEST_ASSERT_EQUAL_UINT16(2026, date->year());
    TEST_ASSERT_EQUAL_UINT8(4, date->month());
    TEST_ASSERT_EQUAL_UINT8(24, date->day());

    // Time 000001.00 → 00:00:01
    const auto time = gps.time.consume();
    TEST_ASSERT_TRUE(time.has_value());
    TEST_ASSERT_EQUAL_UINT8(0, time->hour());
    TEST_ASSERT_EQUAL_UINT8(0, time->minute());
    TEST_ASSERT_EQUAL_UINT8(1, time->second());
}

void run_satellites_tests(void)
{
    RUN_TEST(test_satellites_single_gpgsa_full);
    RUN_TEST(test_satellites_gsa_partial_prns);
    RUN_TEST(test_satellites_gsa_legacy_no_system_id);
    RUN_TEST(test_satellites_multi_constellation_accumulation);
    RUN_TEST(test_satellites_per_system_replacement);
    RUN_TEST(test_satellites_talker_derived_system_id);
    RUN_TEST(test_satellites_bad_checksum_no_commit);
    RUN_TEST(test_satellites_used_count_from_gga);
    RUN_TEST(test_satellites_real_um980_5_constellations);
    RUN_TEST(test_satellites_real_um980_full_epoch_with_gga_rmc);
}
