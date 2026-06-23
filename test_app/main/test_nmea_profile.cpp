#include "unity.h"
#include "gpstask.h"

#include <cstdio>
#include <string_view>
#include <esp_timer.h>
#include <esp_log.h>

// The sample stream is embedded at build time via EMBED_TXTFILES in
// CMakeLists.txt. EMBED_TXTFILES appends a NUL terminator and `_end` points
// just past it, so the original byte count is (end - start - 1).
extern const char s_nmea_start[] asm("_binary_large_nmea_stream_example_16_epochs_txt_start");
extern const char s_nmea_end[]   asm("_binary_large_nmea_stream_example_16_epochs_txt_end");

namespace {

// Passes over the whole stream; averaged to absorb timer jitter and one-time
// costs (e.g. the first clock-discipline settimeofday()).
constexpr int PROFILE_PASSES = 20;

std::string_view sampleStream()
{
    const auto len = static_cast<size_t>(s_nmea_end - s_nmea_start - 1);
    return std::string_view{s_nmea_start, len};
}

// Count complete GNSS epochs in the stream. Each epoch carries exactly one
// Unicore #PPPNAVA solution message (also the gate for processNewLocation), so
// counting that marker yields the epoch count without hardcoding it.
size_t countEpochs(std::string_view stream)
{
    constexpr std::string_view marker = "#PPPNAVA";
    size_t count = 0;
    for (size_t pos = stream.find(marker); pos != std::string_view::npos;
         pos = stream.find(marker, pos + marker.size()))
        ++count;
    return count;
}

// processNewLocation()/configure logging is very chatty (many ESP_LOGI per
// epoch). Over PROFILE_PASSES * epochs calls that UART I/O would dominate the
// measurement, so silence the GPS tags while timing. Production still emits
// these logs, but only at the 1 Hz epoch cadence.
void muteGpsLogs(esp_log_level_t level)
{
    esp_log_level_set("read-gps-location", level);
    esp_log_level_set("gps logger", level);
    esp_log_level_set("GPS_TASK", level);
}

// Per-epoch microseconds from each test, shared so run_nmea_profile_tests()
// can report the downstream (full - parse-only) delta.
double s_fullUsPerEpoch = 0.0;
double s_parseUsPerEpoch = 0.0;

double report(const char *label, std::string_view nmea, size_t epochsPerPass, int64_t totalUs)
{
    const double totalEpochs = static_cast<double>(PROFILE_PASSES) * epochsPerPass;
    const double bytes       = static_cast<double>(nmea.size());
    const double usPerEpoch  = static_cast<double>(totalUs) / totalEpochs;
    const double nsPerByte   = static_cast<double>(totalUs) * 1000.0
                             / (static_cast<double>(PROFILE_PASSES) * bytes);
    const double bytesPerSec = totalUs > 0
                             ? bytes * PROFILE_PASSES * 1e6 / static_cast<double>(totalUs) : 0.0;

    std::printf("[NMEA-PROFILE] %-11s | %.0f bytes/epoch | %8.1f us/epoch | "
                "%6.1f ns/byte | %9.0f bytes/s (avg of %.0f epochs)\n",
                label, bytes / static_cast<double>(epochsPerPass), usPerEpoch,
                nsPerByte, bytesPerSec, totalEpochs);
    std::fflush(stdout);
    return usPerEpoch;
}

} // namespace

// ── Test A: full per-epoch pipeline (encode + processNewLocation) ────────────

static void test_profile_full_pipeline(void)
{
    const std::string_view nmea = sampleStream();
    const size_t epochs = countEpochs(nmea);
    TEST_ASSERT_GREATER_THAN(0u, epochs);

    int gnssEvents = 0;
    GpsTask task;
    TEST_ASSERT_EQUAL(ESP_OK, task.configureTinyGps());
    task.configureGnssEvent([&gnssEvents](std::string_view) { ++gnssEvents; });

    muteGpsLogs(ESP_LOG_ERROR);
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < PROFILE_PASSES; ++i)
        task.feed(nmea);
    const int64_t t1 = esp_timer_get_time();
    muteGpsLogs(ESP_LOG_INFO);

    s_fullUsPerEpoch = report("full", nmea, epochs, t1 - t0);
    std::printf("[NMEA-PROFILE] full: %zu epochs/pass, %d GNSS fixes emitted over %d passes\n",
                epochs, gnssEvents, PROFILE_PASSES);
    std::fflush(stdout);

    // A broken parse must not masquerade as "fast": confirm the epochs were
    // consumed and processNewLocation produced GNSS fixes (the heavy path ran).
    TEST_ASSERT_GREATER_THAN(0u, task.charsProcessed());
    TEST_ASSERT_GREATER_THAN(0, gnssEvents);
}

// ── Test B: parse-only (encode, no downstream work) ──────────────────────────

static void test_profile_parse_only(void)
{
    const std::string_view nmea = sampleStream();
    const size_t epochs = countEpochs(nmea);
    TEST_ASSERT_GREATER_THAN(0u, epochs);

    GpsTask task;
    TEST_ASSERT_EQUAL(ESP_OK, task.configureTinyGps());

    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < PROFILE_PASSES; ++i)
        task.encodeOnly(nmea);
    const int64_t t1 = esp_timer_get_time();

    TEST_ASSERT_GREATER_THAN(0u, task.charsProcessed());

    s_parseUsPerEpoch = report("parse-only", nmea, epochs, t1 - t0);
}

void run_nmea_profile_tests(void)
{
    RUN_TEST(test_profile_parse_only);
    RUN_TEST(test_profile_full_pipeline);

    // Downstream cost = full pipeline minus the parse it contains.
    const double downstreamUs = s_fullUsPerEpoch - s_parseUsPerEpoch;
    std::printf("[NMEA-PROFILE] %-11s | %8.1f us/epoch (processNewLocation: "
                "formatting + Qstarz + clock discipline)\n",
                "downstream", downstreamUs);
    std::fflush(stdout);
}
