#pragma once

#include <array>
#include <string>
#include <string_view>
#include <memory>
#include <chrono>
#include <optional>
#include <vector>
#include <cstddef>
#include <compare>
#include <flat_set>
#include <atomic>
#include <functional>
#include <expected>
#include <driver/gpio.h>
#include <driver/uart.h>
#include "TinyGPSPlus.h"    /// TinyGPSLocation
#include "unicore.h"        /// PPPInfo

struct PppInfo;

struct SatelliteInfo {
    uint8_t      prn       = 0;
    GnssSystemId systemId  = GnssSystemId::Unknown;
    int16_t      elevation = -1;   // degrees; -1 = unknown
    int16_t      azimuth   = -1;   // degrees
    int16_t      cn0       = -1;   // dB-Hz; -1 = no signal

    auto operator<=>(const SatelliteInfo&) const = default;
};

struct GpsInfo
{
    std::flat_set<SatelliteInfo> satellites;   // satellites used in fix, from all GNGSA
    std::size_t satellitesInView = 0;          // count of satellites in view, from all GNGSV
    std::chrono::system_clock::time_point worldTime;
    std::optional<double> lat;
    std::optional<double> lon;
    std::optional<double> altitude;
    std::optional<double> geoidAlt;
    std::optional<double> fixHDOP;
    std::optional<double> fixVDOP;
    std::optional<double> fixPDOP;
    std::optional<TinyGPSLocation::GnssQuality> quality;
    std::optional<TinyGPSLocation::PositioningMode> positioningMode;
    std::optional<TinyGPSLocation::Fix2D3DMode> fix2D3DMode;
    std::optional<TinyGPSLocation::Fix2D3DType> fix2D3DType;
};

class GpsTask
{
public:
    static constexpr uint32_t GPS_TASK_DELAY_MS = 20;
    static constexpr uint32_t GPS_TASK_TX2RX_DELAY_MICROSEC = 100;
    static constexpr uint32_t GPS_TASK_REBOOT_DELAY_MICROSEC = 6 * 1000;
    static constexpr int64_t MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC = 20;
    static constexpr int64_t RTC_RESYNC_MIN_DELTA_SEC = 3;
    static constexpr int UART_TX_GPIO_PIN = GPIO_NUM_16;
    static constexpr int UART_RX_GPIO_PIN = GPIO_NUM_17;
    static constexpr int CONFIGS_MAX_ITERATIONS = 1000;
    static constexpr std::array<int, 8> UM980_BAUD_RATES = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    static constexpr int UM980_UART_BAUDRATE = 460800;
    static constexpr int RX_BUF_SIZE = 1024;
    static constexpr uart_port_t GPS_UART_PORT = UART_NUM_1;

    /// init UART and structures
    [[nodiscard("GPS UART misconfigured if unchecked")]]
    esp_err_t configureUart(int uart_speed);

    /// configure GPS module (internal settings, modes, etc.)
    /// call AFTER task start to see UART output
    [[nodiscard("GPS module misconfigured if unchecked")]]
    esp_err_t configureUM980();

    /// configure TinyGPS library
    [[nodiscard("GPS parser misconfigured if unchecked")]]
    esp_err_t configureTinyGps();

    using QStarZPackets = std::array<std::vector<std::byte>, 4>;
    using NmeaStringReadyEvent = std::function<void(std::string_view nmea)>;
    using GnssLogReadyEvent = std::function<void(std::string_view gnssLog)>;
    using QStarZPacketsReadyEvent = std::function<void(const QStarZPackets &packets)>;

    void configureNmeaEvent(NmeaStringReadyEvent event);
    void configureGnssEvent(GnssLogReadyEvent event);
    void configureQStarZEvent(QStarZPacketsReadyEvent event);

    void executeTask();
    void terminate();

    /// Drive the production per-epoch pipeline (encode + processNewLocation)
    /// over an in-memory NMEA buffer, bypassing the UART. Used by profiling and
    /// for offline testing of the parse pipeline.
    void feed(std::string_view data);

    /// Parse-only variant of feed(): encodes characters without running the
    /// downstream processNewLocation() work. Used to isolate parse cost.
    void encodeOnly(std::string_view data);

    /// Number of characters fed into the NMEA parser so far.
    uint32_t charsProcessed() const { return m_gps.charsProcessed(); }

    static bool hasLock(const GpsInfo &info);
    static bool has3DLock(const GpsInfo &info);
    bool processNewLocation();
    int sendData(std::string_view data);
    std::expected<std::string, esp_err_t> sendStringAndWait(std::string_view data);

    static void dopToMeters(std::string& out, const uint32_t dop) noexcept;
    static std::string printGpsTimeInfo(const GpsInfo &p);
    static std::string printGpsGeoInfo(const GpsInfo &p);
    static QStarZPackets emulateQstarzBinary(const GpsInfo &p);
    static std::string printPppTimeInfo(const PppInfo &p);
    static std::string printPppGeoInfo(const PppInfo &p, const double &gnssToPppDistance);
    static double geoDistance(const double &lat1, const double &lon1, const double &lat2, const double &lon2);

    GpsTask() = default;
    GpsTask(const GpsTask &) = delete("GpsTask owns a UART port handle — copying aliases hardware resources");
    GpsTask &operator=(const GpsTask &) = delete("GpsTask owns a UART port handle — copying aliases hardware resources");

private:
    static constexpr std::string_view NMEA_MSG_GXGGA = "GNGGA";       // GGA message (GPGGA, GNGGA etc)
    static constexpr std::string_view UNICORE_MSG_PPPNAV = "PPPNAVA"; // Unicore protocol, PPP navigation solution

    std::atomic<bool> m_terminateASAP{false};
    NmeaStringReadyEvent m_nmeaEvent;
    GnssLogReadyEvent m_gnssEvent;
    QStarZPacketsReadyEvent m_qstarzEvent;
    TinyGPSPlus m_gps;

    TinyGPSCustom ggaEpoch;                          // detects each new GGA (epoch boundary)
    TinyGPSCustom pppnavWeek;
    TinyGPSCustom pppnavSecsOFWeek;
    TinyGPSCustom pppnavLeapSecs;
    TinyGPSCustom pppnavSolStatus;  // custom extract 'Solution Status' from PPPNAVA, element # ???
    TinyGPSCustom pppnavPosType;    // custom extract 'Position Status' from PPPNAVA, element # ???
    TinyGPSCustom pppnavLat;
    TinyGPSCustom pppnavLon;
    TinyGPSCustom pppnavAlt;
    TinyGPSCustom pppnavLatStdDev;
    TinyGPSCustom pppnavLonStdDev;
    TinyGPSCustom pppnavAltStdDev;
    TinyGPSCustom pppnavSolAge;  // custom extract 'Solution Status' from PPPNAVA, element # ???
    TinyGPSCustom pppnavSatellites;
    TinyGPSCustom pppnavDatumId;
    TinyGPSCustom pppnavStationId; /// can be converted to System (B2b, E6-HAS, etc)
    
    static void logNmeaMessageToSd(std::string_view msg);

    [[nodiscard("UART misconfigured if unchecked")]]
    esp_err_t setupUartToUM980();

    [[nodiscard("UART misconfigured if unchecked")]]
    esp_err_t testGnssBaudRate(int uart_speed);

    /// default delay between send and receive
    void gpsUartDelay();
    void readFromUart(std::string &newData);
};