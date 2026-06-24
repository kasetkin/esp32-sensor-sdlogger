#include "fake_nmea.h"

#include <sstream>
#include <string_view>
#include <esp_log.h>

// extern const char s_nmea_start[] asm("_binary_large_nmea_stream_example_16_epochs_txt_start");
// extern const char s_nmea_end[]   asm("_binary_large_nmea_stream_example_16_epochs_txt_end");
extern const char s_nmea_start[] asm("_binary_sats_in_view_nmea_txt_start");
extern const char s_nmea_end[] asm("_binary_sats_in_view_nmea_txt_end");

static const size_t len = static_cast<size_t>(s_nmea_end - s_nmea_start - 1);
static const std::string_view fakeNmeaLog{s_nmea_start, len};
static size_t stringPosition = 0;

void fakeNmeaLine(std::string &line)
{
    ESP_LOGD("fakenmea", "request for next line from position %zu, full NMEA log size %zu", stringPosition, fakeNmeaLog.size());
    line.clear();
    constexpr size_t MAX_LINE_SIZE = 10000;
    const size_t startPos = stringPosition;
    const size_t maxPos = std::min(startPos + MAX_LINE_SIZE, fakeNmeaLog.size());
    size_t endPos = std::string::npos;
    for (size_t i = startPos + 1; i < maxPos; ++i) {
        if (fakeNmeaLog[i] == '\n') {
            endPos = i;
            break;
        }
    }

    ESP_LOGD("fakenmea", "new line positions: %zu to %zu", startPos, endPos);

    if (endPos == std::string::npos)
        endPos = fakeNmeaLog.size() - 1;

    if (endPos < startPos) {
        ESP_LOGE("fakenmea", "error in logic, start = %zu, end = %zu", startPos, endPos);
        stringPosition = 0;
        return;
    }
        
    const size_t newSize = endPos - startPos + 1;
    line.clear();
    line.reserve(newSize);
    std::string_view resultRange(&fakeNmeaLog[startPos], newSize);
    line.append_range(resultRange);

    ESP_LOGD("fakenmea", "line is %s", line.c_str());
    stringPosition = endPos + 1;
    if (stringPosition >= fakeNmeaLog.size())
        stringPosition = 0;

    ESP_LOGD("fakenmea", "new stringPosition: %zu", stringPosition);
}