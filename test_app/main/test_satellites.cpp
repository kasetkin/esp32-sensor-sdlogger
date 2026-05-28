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

// ── Helpers for the sats[] + bitset-flags Data layout ───────────────────────

// Index of (prn, sys) in d.all(), or d.all().size() if not present.
static std::size_t findSat(const TinyGPSSatellites::Data& d, uint8_t prn, GnssSystemId sys)
{
    const auto all = d.all();
    for (std::size_t i = 0; i < all.size(); ++i)
        if (all[i].prn == prn && all[i].systemId == sys)
            return i;
    return all.size();
}

static int countInView(const TinyGPSSatellites::Data& d, GnssSystemId sys)
{
    int n = 0;
    const auto all = d.all();
    for (std::size_t i = 0; i < all.size(); ++i)
        if (d.inView(i) && all[i].systemId == sys) ++n;
    return n;
}

static int countInSolution(const TinyGPSSatellites::Data& d, GnssSystemId sys)
{
    int n = 0;
    const auto all = d.all();
    for (std::size_t i = 0; i < all.size(); ++i)
        if (d.inSolution(i) && all[i].systemId == sys) ++n;
    return n;
}

// ── Single GPS GSA (NMEA 4.10) ──────────────────────────────────────────────

static void test_satellites_single_gpgsa_full(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GPGSA,A,3,3,4,7,8,15,17,19,22,24,29,31,32,1.0,1.2,1.5,1");

    TEST_ASSERT_TRUE(gps.satellites.isGsaUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_FALSE(gps.satellites.isGsaUpdated()); // consume cleared it

    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixMode::Auto), static_cast<int>(data->mode));
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix3D), static_cast<int>(data->fixType));
    TEST_ASSERT_EQUAL_INT32(100, data->pdop);
    TEST_ASSERT_EQUAL_INT32(120, data->hdop);
    TEST_ASSERT_EQUAL_INT32(150, data->vdop);

    const auto sats = data->all();
    TEST_ASSERT_EQUAL(12u, data->inSolutionCount());
    TEST_ASSERT_EQUAL(12u, sats.size());
    TEST_ASSERT_EQUAL_UINT8(3, sats.front().prn);
    TEST_ASSERT_EQUAL_UINT8(32, sats.back().prn);
    for (std::size_t i = 0; i < sats.size(); ++i) {
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::GPS), static_cast<int>(sats[i].systemId));
        TEST_ASSERT_TRUE(data->inSolution(i));
    }
}

// ── GSA with empty PRN slots ────────────────────────────────────────────────

static void test_satellites_gsa_partial_prns(void)
{
    TinyGPSPlus gps;
    // Only 3 PRNs filled; remaining 9 PRN slots empty
    feedSentence(gps, "GPGSA,A,2,3,4,7,,,,,,,,,,1.0,1.2,1.5,1");

    TEST_ASSERT_TRUE(gps.satellites.isGsaUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix2D), static_cast<int>(data->fixType));

    const auto sats = data->all();
    TEST_ASSERT_EQUAL(3u, data->inSolutionCount());
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

    TEST_ASSERT_TRUE(gps.satellites.isGsaUpdated());

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->all();
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
        TEST_ASSERT_EQUAL(5u, data->inSolutionCount());
    }

    // GLONASS (System ID 2): PRNs 65,66,67
    feedSentence(gps, "GNGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3,2");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(8u, data->inSolutionCount());
    }

    // Galileo (System ID 3): PRNs 5,9
    feedSentence(gps, "GNGSA,A,3,5,9,,,,,,,,,,,1.1,1.2,1.3,3");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->all();
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

// ── New epoch (RMC) clears constellations no longer reported ─────────────────

static void test_satellites_new_epoch_clears_dropped(void)
{
    TinyGPSPlus gps;

    // Epoch 1: GPS + GLONASS reported (in solution and in view).
    feedSentence(gps, "GNRMC,000001.00,A,2530.0,N,13045.0,E,0,0,240426,,,A");
    feedSentence(gps, "GNGSA,A,3,3,4,7,,,,,,,,,,1.1,1.2,1.3,1");
    feedSentence(gps, "GNGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3,2");
    feedSentence(gps, "GPGSV,1,1,03,03,40,100,40,04,30,110,38,07,20,120,35,1");
    feedSentence(gps, "GLGSV,1,1,03,65,40,200,42,66,30,210,39,67,20,220,33,1");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(6u, data->inSolutionCount());   // 3 GPS + 3 GLONASS in solution
    }

    // Epoch 2: a new RMC clears the buffer; only GPS is reported this time.
    feedSentence(gps, "GNRMC,000002.00,A,2530.0,N,13045.0,E,0,0,240426,,,A");
    feedSentence(gps, "GNGSA,A,3,10,11,12,13,,,,,,,,,1.1,1.2,1.3,1");
    feedSentence(gps, "GPGSV,1,1,04,10,40,100,40,11,30,110,38,12,20,120,35,13,10,130,30,1");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->all();
    TEST_ASSERT_EQUAL(4u, sats.size());                     // only the new GPS list

    bool sawOldGpsPrn3 = false;
    bool sawNewGpsPrn10 = false;
    bool sawGlonass = false;
    for (const auto& s : sats) {
        if (s.systemId == GnssSystemId::GPS && s.prn == 3)  sawOldGpsPrn3 = true;
        if (s.systemId == GnssSystemId::GPS && s.prn == 10) sawNewGpsPrn10 = true;
        if (s.systemId == GnssSystemId::GLONASS)            sawGlonass = true;
    }
    TEST_ASSERT_FALSE(sawOldGpsPrn3);   // previous epoch's GPS list gone
    TEST_ASSERT_TRUE(sawNewGpsPrn10);   // new GPS list present
    TEST_ASSERT_FALSE(sawGlonass);      // GLONASS dropped (not reported this epoch)
}

// ── Talker-derived SystemID (GLGSA, no System ID field) ─────────────────────

static void test_satellites_talker_derived_system_id(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GLGSA,A,3,65,66,67,,,,,,,,,,1.1,1.2,1.3");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto sats = data->all();
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
    TEST_ASSERT_TRUE(gps.satellites.isGsaUpdated());
    {
        const auto primed = gps.satellites.consume(); // clears isUpdated
        TEST_ASSERT_TRUE(primed.has_value());
        TEST_ASSERT_EQUAL(3u, primed->inSolutionCount());
    }
    TEST_ASSERT_FALSE(gps.satellites.isGsaUpdated());

    // Now feed a sentence with deliberately wrong checksum
    feedSentenceBadChecksum(gps, "GPGSA,A,3,9,10,11,12,13,,,,,,,,1.0,1.0,1.0,1");

    TEST_ASSERT_FALSE(gps.satellites.isGsaUpdated());   // commit never fired
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
    const auto sats = data->all();
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
            case GnssSystemId::NavIC:   break;
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
    feedSentence(gps, kUm980EpochRmc);   // RMC leads the epoch and clears the buffer
    feedSentence(gps, kUm980EpochGga);
    for (const auto sentence : kUm980EpochGsas)
        feedSentence(gps, sentence);

    // GGA's satellites-used field matches the GSA union (sanity cross-check)
    const auto sats = gps.satellites.consume();
    const auto used = gps.satellitesUsedCount.consume();
    TEST_ASSERT_TRUE(sats.has_value());
    TEST_ASSERT_TRUE(used.has_value());
    TEST_ASSERT_EQUAL_UINT32(27, used->raw);
    TEST_ASSERT_EQUAL(27u, sats->inSolutionCount());

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

// ── GSV: single GPS group (three sentences, 12 satellites in view) ───────────

static void test_satellites_gsv_single_gps_group(void)
{
    TinyGPSPlus gps;
    // From main/fake_nmea.cpp (NMEA 4.10, no trailing signal ID).
    feedSentence(gps, "GPGSV,3,1,12,03,57,094,40,04,33,056,36,07,27,116,33,08,53,249,40");
    feedSentence(gps, "GPGSV,3,2,12,15,68,328,32,17,46,115,34,19,05,327,39,22,11,053,39");
    feedSentence(gps, "GPGSV,3,3,12,24,52,269,31,29,46,288,36,31,88,250,38,32,55,058,32");
    TEST_ASSERT_TRUE(gps.satellites.isInViewUpdated());    // GSV data available (per-sentence)

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto view = data->all();
    TEST_ASSERT_EQUAL(12u, data->inViewCount());
    TEST_ASSERT_EQUAL(12u, view.size());

    // First quad of the first sentence.
    TEST_ASSERT_EQUAL_UINT8(3, view.front().prn);
    TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::GPS), static_cast<int>(view.front().systemId));
    TEST_ASSERT_EQUAL_INT16(57, view.front().elevationDeg);
    TEST_ASSERT_EQUAL_INT16(94, view.front().azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(40, view.front().cn0DbHz);

    // Last quad of the last sentence.
    TEST_ASSERT_EQUAL_UINT8(32, view.back().prn);
    TEST_ASSERT_EQUAL_INT16(55, view.back().elevationDeg);
    TEST_ASSERT_EQUAL_INT16(58, view.back().azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(32, view.back().cn0DbHz);

    // No GSA fed → nothing in solution.
    TEST_ASSERT_EQUAL(0u, data->inSolutionCount());
    for (std::size_t i = 0; i < view.size(); ++i)
        TEST_ASSERT_FALSE(data->inSolution(i));
}

// ── GSV: an empty SNR field leaves C/N0 unset ───────────────────────────────

static void test_satellites_gsv_empty_snr(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GPGSV,1,1,01,05,10,123,");   // trailing SNR field empty (not tracking)

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto view = data->all();
    TEST_ASSERT_EQUAL(1u, view.size());
    TEST_ASSERT_EQUAL_UINT8(5, view.front().prn);
    TEST_ASSERT_EQUAL_INT16(10, view.front().elevationDeg);
    TEST_ASSERT_EQUAL_INT16(123, view.front().azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(-1, view.front().cn0DbHz);   // empty SNR → -1 (not reported)
}

// ── GSV: multiple constellations land in their own systems ──────────────────

static void test_satellites_gsv_multi_constellation(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GPGSV,1,1,02,03,57,094,40,04,33,056,36");
    feedSentence(gps, "GLGSV,1,1,02,65,40,100,35,66,20,200,30");
    feedSentence(gps, "GAGSV,1,1,01,05,45,150,38");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());

    const auto view = data->all();
    TEST_ASSERT_EQUAL(5u, view.size());

    int gpsN = 0, gloN = 0, galN = 0;
    for (const auto& s : view) {
        switch (s.systemId) {
            case GnssSystemId::GPS:     ++gpsN; break;
            case GnssSystemId::GLONASS: ++gloN; break;
            case GnssSystemId::Galileo: ++galN; break;
            default: break;
        }
    }
    TEST_ASSERT_EQUAL(2, gpsN);
    TEST_ASSERT_EQUAL(2, gloN);
    TEST_ASSERT_EQUAL(1, galN);
}

// ── GSV: the same PRN on multiple signals is deduped, strongest C/N0 kept ────

static void test_satellites_gsv_signal_dedupe(void)
{
    TinyGPSPlus gps;
    // Epoch 1: PRN 7 on signal 1 (C/N0 25), then signal 5 (C/N0 42).
    feedSentence(gps, "GPGSV,1,1,01,07,30,100,25,1");
    feedSentence(gps, "GPGSV,1,1,01,07,30,100,42,5");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        const auto view = data->all();
        TEST_ASSERT_EQUAL(1u, view.size());                 // one row, not two
        TEST_ASSERT_EQUAL_UINT8(7, view.front().prn);
        TEST_ASSERT_EQUAL_INT16(42, view.front().cn0DbHz);  // stronger signal kept
    }

    // Epoch 2 (a new RMC clears the buffer): stronger comes first; a later weaker
    // signal must not overwrite it.
    feedSentence(gps, "GNRMC,000002.00,A,2530.0,N,13045.0,E,0,0,240426,,,A");
    feedSentence(gps, "GPGSV,1,1,01,07,30,100,42,1");
    feedSentence(gps, "GPGSV,1,1,01,07,30,100,25,5");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        const auto view = data->all();
        TEST_ASSERT_EQUAL(1u, view.size());
        TEST_ASSERT_EQUAL_INT16(42, view.front().cn0DbHz);  // strongest still wins
    }
}

// ── GSV: a new epoch replaces the previous in-view list (no stale build-up) ──

static void test_satellites_gsv_epoch_reset(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GPGSV,1,1,02,03,57,094,40,04,33,056,36");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        TEST_ASSERT_EQUAL(2u, data->all().size());
    }
    // New epoch: a fresh RMC clears the buffer before the new GSV arrives.
    feedSentence(gps, "GNRMC,000002.00,A,2530.0,N,13045.0,E,0,0,240426,,,A");
    feedSentence(gps, "GPGSV,1,1,02,10,57,094,40,11,33,056,36");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(2u, view.size());
    for (const auto& s : view)
        TEST_ASSERT_TRUE(s.prn == 10 || s.prn == 11);       // old 3/4 are gone
}

// ── GSV ∪ GSA: in-solution sats enriched, in-view sats flagged ──────────────

static void test_satellites_gsv_gsa_join(void)
{
    TinyGPSPlus gps;
    // In view: PRNs 3,4,7,8 with elevation/azimuth/C/N0.
    feedSentence(gps, "GPGSV,1,1,04,03,57,094,40,04,33,056,36,07,27,116,33,08,53,249,40");
    TEST_ASSERT_FALSE(gps.satellites.isUpdated());          // GSA not yet seen
    // In solution: PRNs 3,4,7 (PRN 8 is in view only).
    feedSentence(gps, "GPGSA,A,2,3,4,7,,,,,,,,,,1.0,1.2,1.5,1");
    TEST_ASSERT_TRUE(gps.satellites.isUpdated());           // both sources fresh now

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_FALSE(gps.satellites.isUpdated());          // consume cleared both

    // In solution: 3 PRNs; the GSV data enriched the matching record.
    TEST_ASSERT_EQUAL(3u, data->inSolutionCount());
    const std::size_t i3 = findSat(*data, 3, GnssSystemId::GPS);
    TEST_ASSERT_TRUE(i3 < data->all().size());
    TEST_ASSERT_TRUE(data->inSolution(i3));
    const auto& s3 = data->all()[i3];
    TEST_ASSERT_EQUAL_INT16(57, s3.elevationDeg);
    TEST_ASSERT_EQUAL_INT16(94, s3.azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(40, s3.cn0DbHz);

    // In view: 4 sats; 3/4/7 are in solution, 8 is in view only.
    TEST_ASSERT_EQUAL(4u, data->inViewCount());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(4u, view.size());
    for (std::size_t i = 0; i < view.size(); ++i) {
        TEST_ASSERT_TRUE(data->inView(i));
        const bool expectInSolution = (view[i].prn != 8);
        TEST_ASSERT_EQUAL_INT(expectInSolution ? 1 : 0, data->inSolution(i) ? 1 : 0);
    }
}

// ── GSV: QZSS talker (GQ) is recognised (talker-suffix fix) ─────────────────

static void test_satellites_gsv_qzss_talker(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GQGSV,1,1,01,01,45,180,40");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(1u, view.size());
    TEST_ASSERT_EQUAL_UINT8(1, view.front().prn);
    TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::QZSS), static_cast<int>(view.front().systemId));
}

// ── GSV: a bad-checksum sentence is dropped; good sentences are kept ─────────

static void test_satellites_gsv_bad_checksum_no_pollution(void)
{
    TinyGPSPlus gps;
    // A good GSV sentence's satellites are upserted; a bad-checksum one is dropped.
    feedSentence(gps, "GPGSV,1,1,02,10,40,200,30,11,20,100,28");           // good: PRN 10,11
    TEST_ASSERT_TRUE(gps.satellites.isInViewUpdated());
    feedSentenceBadChecksum(gps, "GPGSV,1,1,02,03,57,094,40,04,33,056,36"); // bad: PRN 3,4 dropped

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(2u, view.size());
    for (const auto& s : view)
        TEST_ASSERT_TRUE(s.prn == 10 || s.prn == 11);       // 3/4 from the bad sentence never added
}

// ── Freshness: isUpdated() requires BOTH GSA and GSV ────────────────────────

static void test_satellites_updated_requires_gsa_and_gsv(void)
{
    TinyGPSPlus gps;

    feedSentence(gps, "GPGSA,A,3,3,4,7,,,,,,,,,,1.0,1.2,1.5,1");
    TEST_ASSERT_TRUE(gps.satellites.isGsaUpdated());
    TEST_ASSERT_FALSE(gps.satellites.isInViewUpdated());
    TEST_ASSERT_FALSE(gps.satellites.isUpdated());          // GSV still missing

    feedSentence(gps, "GPGSV,1,1,03,03,57,094,40,04,33,056,36,07,27,116,33");
    TEST_ASSERT_TRUE(gps.satellites.isInViewUpdated());
    TEST_ASSERT_TRUE(gps.satellites.isUpdated());           // both fresh

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_FALSE(gps.satellites.isUpdated());          // both cleared
    TEST_ASSERT_FALSE(gps.satellites.isGsaUpdated());
    TEST_ASSERT_FALSE(gps.satellites.isInViewUpdated());
}

// ── NavIC / IRNSS: GIGSV satellites are tagged NavIC ────────────────────────

static void test_satellites_gsv_navic(void)
{
    TinyGPSPlus gps;
    feedSentence(gps, "GIGSV,1,1,02,40,34,209,32,41,40,170,34,6");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(2u, view.size());
    for (const auto& s : view)
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::NavIC), static_cast<int>(s.systemId));
    TEST_ASSERT_EQUAL_UINT8(40, view[0].prn);
    TEST_ASSERT_EQUAL_UINT8(41, view[1].prn);
}

// ── GSA split: two same-System-ID sentences accumulate; a new epoch (RMC) replaces ──

static void test_satellites_gsa_split_accumulation(void)
{
    TinyGPSPlus gps;
    // BeiDou solution split across two System ID 4 GSA sentences: 12 + 3 = 15.
    feedSentence(gps, "GNGSA,M,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,0.8,0.9,4");
    feedSentence(gps, "GNGSA,M,3,13,14,15,,,,,,,,,,1.0,0.8,0.9,4");
    {
        const auto data = gps.satellites.consume();
        TEST_ASSERT_TRUE(data.has_value());
        const auto sol = data->all();
        TEST_ASSERT_EQUAL(15u, sol.size());
        for (const auto& s : sol)
            TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::BeiDou), static_cast<int>(s.systemId));
    }

    // A new epoch (RMC) clears the buffer; the next System ID 4 GSA stands alone.
    feedSentence(gps, "GNRMC,000002.00,A,2530.0,N,13045.0,E,0,0,240426,,,A");
    feedSentence(gps, "GNGSA,M,3,20,21,,,,,,,,,,,1.0,0.8,0.9,4");
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto sol = data->all();
    TEST_ASSERT_EQUAL(2u, sol.size());                           // cleared, not appended (not 17)
    TEST_ASSERT_EQUAL_UINT8(20, sol[0].prn);
    TEST_ASSERT_EQUAL_UINT8(21, sol[1].prn);
}

// ── Real UM980 capture: one full epoch from docs/2026-04-26-sdlogger_UM980_nmea.csv ──
// Middle-of-file cycle at t = 083621.00 UTC, all six constellations. PPPNAVA is
// omitted (its 32-bit CRC doesn't fit feedSentence). Lat/lon scrubbed to a synthetic
// mid-Pacific coordinate; all other fields are verbatim from the recording.
//   In view (GSV):     GPS 9, GLONASS 8, BeiDou 16, Galileo 12, QZSS 3, NavIC 5 = 53
//   In solution (GSA): GPS 7, GLONASS 5, Galileo 10, BeiDou 13 (12+1 split), QZSS 2 = 37
//                      == GGA satellites-used (37)

static constexpr std::array<std::string_view, 15> kEpoch0426Gsv = {
    "GPGSV,3,1,09,05,70,058,39,11,18,078,36,26,16,310,31,21,41,050,36,1",
    "GPGSV,3,2,09,25,25,215,38,15,25,159,37,29,78,297,40,20,30,129,35,1",
    "GPGSV,3,3,09,18,32,280,36,1",
    "GLGSV,2,1,08,66,50,070,38,82,53,331,35,65,15,026,22,88,06,097,24,2",
    "GLGSV,2,2,08,80,25,262,30,81,51,063,36,73,17,320,33,67,36,156,33,2",
    "GBGSV,4,1,16,66,30,134,35,67,41,175,33,02,41,175,36,03,30,134,35,4",
    "GBGSV,4,2,16,01,10,108,35,38,41,104,39,42,34,051,35,06,70,060,39,4",
    "GBGSV,4,3,16,09,79,174,38,13,67,084,40,41,22,184,38,27,31,298,37,4",
    "GBGSV,4,4,16,14,13,037,29,08,27,127,33,33,56,132,40,28,73,328,40,4",
    "GAGSV,3,1,12,36,34,272,35,10,56,051,37,12,33,065,30,06,14,143,29,3",
    "GAGSV,3,2,12,25,45,235,36,16,18,186,31,11,73,345,33,04,23,118,30,3",
    "GAGSV,3,3,12,19,21,060,27,28,,,34,02,30,304,34,18,,,40,3",
    "GQGSV,1,1,03,03,29,083,36,07,19,119,40,55,38,160,35,5",
    "GIGSV,2,1,05,40,34,209,32,41,40,170,34,10,16,118,40,09,19,199,33,6",
    "GIGSV,2,2,05,02,48,213,42,6",
};
static constexpr std::array<std::string_view, 6> kEpoch0426Gsa = {
    "GNGSA,M,3,05,11,21,25,29,20,18,,,,,,0.9,0.5,0.7,1",
    "GNGSA,M,3,66,82,88,81,67,,,,,,,,0.9,0.5,0.7,2",
    "GNGSA,M,3,36,10,12,06,25,16,11,04,19,02,,,0.9,0.5,0.7,3",
    "GNGSA,M,3,02,03,38,42,06,09,13,41,27,14,08,33,0.9,0.5,0.7,4",
    "GNGSA,M,3,28,,,,,,,,,,,,0.9,0.5,0.7,4",
    "GNGSA,M,3,03,07,,,,,,,,,,,0.9,0.5,0.7,5",
};
// Lat/lon scrubbed to the synthetic mid-Pacific coordinate used by the other real test.
static constexpr std::string_view kEpoch0426Rmc =
    "GNRMC,083621.00,A,2530.12345678,N,13045.87654321,E,2.004,6.7,260426,1.8,E,P,S";
static constexpr std::string_view kEpoch0426Gga =
    "GNGGA,083621.00,2530.12345678,N,13045.87654321,E,5,37,0.5,2155.6685,M,-39.5669,M,9.0,9901";

static void test_satellites_real_um980_epoch_2026_04_26(void)
{
    TinyGPSPlus gps;
    // Real intra-epoch order: RMC -> GGA -> GSA -> GSV (RMC clears the buffer).
    feedSentence(gps, kEpoch0426Rmc);
    feedSentence(gps, kEpoch0426Gga);
    for (const auto s : kEpoch0426Gsa)   // six GSA (BeiDou split across two System-ID-4)
        feedSentence(gps, s);
    for (const auto s : kEpoch0426Gsv)
        feedSentence(gps, s);

    TEST_ASSERT_TRUE(gps.satellites.isUpdated());   // both GSA and GSV are fresh

    const auto used = gps.satellitesUsedCount.consume();
    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    TEST_ASSERT_TRUE(used.has_value());

    // ── In view: 53 across six constellations ──
    const auto sats = data->all();
    TEST_ASSERT_EQUAL(53u, data->inViewCount());
    TEST_ASSERT_EQUAL(9,  countInView(*data, GnssSystemId::GPS));
    TEST_ASSERT_EQUAL(8,  countInView(*data, GnssSystemId::GLONASS));
    TEST_ASSERT_EQUAL(16, countInView(*data, GnssSystemId::BeiDou));
    TEST_ASSERT_EQUAL(12, countInView(*data, GnssSystemId::Galileo));
    TEST_ASSERT_EQUAL(3,  countInView(*data, GnssSystemId::QZSS));
    TEST_ASSERT_EQUAL(5,  countInView(*data, GnssSystemId::NavIC));

    // ── In solution: union 37, equals the GGA satellites-used field ──
    TEST_ASSERT_EQUAL(37u, data->inSolutionCount());
    TEST_ASSERT_EQUAL_UINT32(37, used->raw);
    // BeiDou solution split across two GSA sentences (12 + 1) accumulates to 13.
    TEST_ASSERT_EQUAL(13, countInSolution(*data, GnssSystemId::BeiDou));

    // ── GSV → in-solution enrichment: GPS PRN 05 carries elevation/azimuth/C/N0 ──
    const std::size_t iG5 = findSat(*data, 5, GnssSystemId::GPS);
    TEST_ASSERT_TRUE(iG5 < sats.size());
    TEST_ASSERT_TRUE(data->inSolution(iG5));
    TEST_ASSERT_EQUAL_INT16(70, sats[iG5].elevationDeg);
    TEST_ASSERT_EQUAL_INT16(58, sats[iG5].azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(39, sats[iG5].cn0DbHz);

    // ── NavIC PRN 40 in view with elev/azim/C/N0; not part of the solution ──
    const std::size_t iN40 = findSat(*data, 40, GnssSystemId::NavIC);
    TEST_ASSERT_TRUE(iN40 < sats.size());
    TEST_ASSERT_TRUE(data->inView(iN40));
    TEST_ASSERT_EQUAL_INT16(34,  sats[iN40].elevationDeg);
    TEST_ASSERT_EQUAL_INT16(209, sats[iN40].azimuthDeg);
    TEST_ASSERT_EQUAL_INT16(32,  sats[iN40].cn0DbHz);
    TEST_ASSERT_FALSE(data->inSolution(iN40));

    // ── GSA scalars ──
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixMode::Manual), static_cast<int>(data->mode));
    TEST_ASSERT_EQUAL(static_cast<int>(GsaFixType::Fix3D),  static_cast<int>(data->fixType));
    TEST_ASSERT_EQUAL_INT32(90, data->pdop);
    TEST_ASSERT_EQUAL_INT32(50, data->hdop);
    TEST_ASSERT_EQUAL_INT32(70, data->vdop);

    // ── GGA quality + scrubbed location ──
    const auto loc = gps.location.consume();
    TEST_ASSERT_TRUE(loc.has_value());
    TEST_ASSERT_EQUAL(static_cast<int>(TinyGPSLocation::Quality::FloatRTK),
                      static_cast<int>(loc->fixQuality));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 25.50205761,  loc->latDeg());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 130.76460905, loc->lngDeg());

    // ── Date / time from RMC: 26 Apr 2026, 08:36:21 ──
    const auto date = gps.date.consume();
    const auto time = gps.time.consume();
    TEST_ASSERT_TRUE(date.has_value());
    TEST_ASSERT_TRUE(time.has_value());
    TEST_ASSERT_EQUAL_UINT16(2026, date->year());
    TEST_ASSERT_EQUAL_UINT8(4,  date->month());
    TEST_ASSERT_EQUAL_UINT8(26, date->day());
    TEST_ASSERT_EQUAL_UINT8(8,  time->hour());
    TEST_ASSERT_EQUAL_UINT8(36, time->minute());
    TEST_ASSERT_EQUAL_UINT8(21, time->second());
}

// ── GSV: a PRN on multiple signals (incl. a hex Signal ID) is deduped ───────

static void test_satellites_gsv_multi_signal_hex(void)
{
    TinyGPSPlus gps;
    // BeiDou signal 1 (PRN 10 C/N0 40, PRN 01 C/N0 45), then signal B = hex 11
    // (PRN 10 weaker, PRN 02 new) within one epoch.
    feedSentence(gps, "GBGSV,1,1,02,10,30,201,40,01,34,140,45,1");
    feedSentence(gps, "GBGSV,1,1,02,10,30,201,30,02,33,224,41,B");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(3u, view.size());   // distinct PRNs 10, 01, 02 (not 4)

    const std::size_t i10 = findSat(*data, 10, GnssSystemId::BeiDou);
    TEST_ASSERT_TRUE(i10 < view.size());
    TEST_ASSERT_EQUAL_INT16(40, view[i10].cn0DbHz);   // strongest C/N0 kept (40 > 30)
    TEST_ASSERT_TRUE(findSat(*data, 1, GnssSystemId::BeiDou) < view.size());  // signal-1-only survives
    TEST_ASSERT_TRUE(findSat(*data, 2, GnssSystemId::BeiDou) < view.size());  // signal-B-only added
}

// ── GSV: a large 21-sat, 6-sentence BeiDou group fits the buffer ────────────

static void test_satellites_gsv_large_beidou(void)
{
    TinyGPSPlus gps;
    // From the UM980 manual (§7.1.1.9 GPGSV): BeiDou signal 1, 21 satellites in view.
    feedSentence(gps, "GBGSV,6,1,21,36,72,016,49,19,24,172,36,39,75,082,50,30,13,111,38,1");
    feedSentence(gps, "GBGSV,6,2,21,10,30,201,35,27,10,062,32,01,34,140,40,07,40,195,39,1");
    feedSentence(gps, "GBGSV,6,3,21,16,78,051,49,22,59,233,48,09,69,327,45,59,38,144,43,1");
    feedSentence(gps, "GBGSV,6,4,21,03,42,188,39,04,25,124,36,40,48,180,45,45,41,261,40,1");
    feedSentence(gps, "GBGSV,6,5,21,60,28,227,36,02,33,224,32,46,25,059,35,21,32,308,35,1");
    feedSentence(gps, "GBGSV,6,6,21,06,79,008,47,1");

    const auto data = gps.satellites.consume();
    TEST_ASSERT_TRUE(data.has_value());
    const auto view = data->all();
    TEST_ASSERT_EQUAL(21u, view.size());
    for (const auto& s : view)
        TEST_ASSERT_EQUAL(static_cast<int>(GnssSystemId::BeiDou), static_cast<int>(s.systemId));
}

void run_satellites_tests(void)
{
    RUN_TEST(test_satellites_single_gpgsa_full);
    RUN_TEST(test_satellites_gsa_partial_prns);
    RUN_TEST(test_satellites_gsa_legacy_no_system_id);
    RUN_TEST(test_satellites_multi_constellation_accumulation);
    RUN_TEST(test_satellites_new_epoch_clears_dropped);
    RUN_TEST(test_satellites_talker_derived_system_id);
    RUN_TEST(test_satellites_bad_checksum_no_commit);
    RUN_TEST(test_satellites_used_count_from_gga);
    RUN_TEST(test_satellites_real_um980_5_constellations);
    RUN_TEST(test_satellites_real_um980_full_epoch_with_gga_rmc);
    RUN_TEST(test_satellites_gsv_single_gps_group);
    RUN_TEST(test_satellites_gsv_empty_snr);
    RUN_TEST(test_satellites_gsv_multi_constellation);
    RUN_TEST(test_satellites_gsv_signal_dedupe);
    RUN_TEST(test_satellites_gsv_epoch_reset);
    RUN_TEST(test_satellites_gsv_gsa_join);
    RUN_TEST(test_satellites_gsv_qzss_talker);
    RUN_TEST(test_satellites_gsv_bad_checksum_no_pollution);
    RUN_TEST(test_satellites_updated_requires_gsa_and_gsv);
    RUN_TEST(test_satellites_gsv_navic);
    RUN_TEST(test_satellites_gsa_split_accumulation);
    RUN_TEST(test_satellites_gsv_multi_signal_hex);
    RUN_TEST(test_satellites_gsv_large_beidou);
    RUN_TEST(test_satellites_real_um980_epoch_2026_04_26);
}
