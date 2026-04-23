#pragma once

#include <string>
#include <memory>
#include <chrono>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "TinyGPSPlus.h"    /// TinyGPSLocation
#include "unicore.h"        /// PPPInfo
#include "loggertask.h"
#include "bleservertask.h"

struct PppInfo;

struct GpsInfo
{
    static constexpr double BAD_LATLON = -999999999.0;
    static constexpr double BAD_ALTITUDE = -12000000.0;
    static constexpr uint32_t BAD_DOP = 666000000;

    // TinyGPSLocation location;
    std::chrono::system_clock::time_point worldTime;
    double lat = BAD_LATLON;
    double lon = BAD_LATLON;
    double altitude = BAD_ALTITUDE;
    double geoidAlt = BAD_ALTITUDE;
    uint32_t gsaHDOP = BAD_DOP;
    uint32_t gsaVDOP = BAD_DOP;
    uint32_t gsaPDOP = BAD_DOP;
    TinyGPSLocation::Quality quality = TinyGPSLocation::Quality::Invalid; // quality from GGA
    uint8_t fixType = 0;      // fix type from GPGSA
};

struct QstarzMessage 
{
    /// \todo @claudecode
};

class GpsTask
{
public:
    static constexpr uint32_t GPS_TASK_DELAY_MS = 30;
    static constexpr uint32_t GPS_TASK_TX2RX_DELAY_MICROSEC = 100;
    static constexpr int64_t MAX_GPS_TO_RTC_MAX_TIME_DELTA_SEC = 20;
    static constexpr int UART_TX_GPIO_PIN = GPIO_NUM_16;
    static constexpr int UART_RX_GPIO_PIN = GPIO_NUM_17;
    static constexpr int UM980_UART_BAUDRATE = 115200;
    static constexpr int RX_BUF_SIZE = 1024;
    static constexpr uart_port_t GPS_UART_PORT = UART_NUM_1;

    /// init UART and structures
    esp_err_t configureUart();

    /// configure GPS module (internal settings, modes, etc.)
    /// call AFTER task start to see UART output
    esp_err_t configureUM980();

    /// configure TinyGPS library 
    esp_err_t configureTinyGps();

    esp_err_t setupLogger(std::shared_ptr<LoggerTask> logger);
    esp_err_t setupBleTask(std::shared_ptr<BleSppServerTask> ble);
    void executeTask();

    static bool hasLock(const GpsInfo &info);
    static bool has3DLock(const GpsInfo &info);
    bool processNewLocation();
    int sendData(const char* data);
    int sendStringAndWait(const std::string &str, std::string &reply);
private:
    const char * NMEA_MSG_GXGSA = "GNGSA"; // GSA message (GPGSA, GNGSA etc)
    const char * UNICORE_MSG_PPPNAV = "PPPNAVA"; // Unicore protocol, PPP navigation solution

    std::shared_ptr<LoggerTask> m_logger;
    std::shared_ptr<BleSppServerTask> m_ble;
    TinyGPSPlus m_gps;

    TinyGPSCustom gsafixtype; // custom extract fix type from GPGSA, , GSA element #2
    TinyGPSCustom gsapdop;    // custom extract PDOP from GPGSA, GSA element #15
    TinyGPSCustom gsahdop;    // custom extract HDOP from GPGSA, GSA element #16
    TinyGPSCustom gsavdop;    // custom extract VDOP from GPGSA, GSA element #17
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
    
    static void logNmeaMessageToSd(const std::string &msg);
    static std::string dopToMeters(const uint32_t dop);
    static double geoDistance(const double &lat1, const double &lon1, const double &lat2, const double &lon2);
    static std::string printGpsTimeInfo(const GpsInfo &p);
    static std::string printGpsGeoInfo(const GpsInfo &p);
    static std::string printPppTimeInfo(const PppInfo &p);
    static std::string printPppGeoInfo(const PppInfo &p, const double &gnssToPppDistance);
    static std::array<std::string, 4> emulateQstarzBinary(const GpsInfo &p);

    /// default delay between send and receive
    void gpsUartDelay();
    void readFromUart(std::string &newData);
};