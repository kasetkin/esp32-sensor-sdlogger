#pragma once

#include <string>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "TinyGPSPlus.h"

class GpsTask
{
public:
    static constexpr uint32_t GPS_TASK_DELAY_MS = 30;
    static constexpr uint32_t GPS_TASK_TX2RX_DELAY_MICROSEC = 100;
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
    
    esp_err_t configureTinyGps();
    void executeTask();

    bool hasLock();
    bool has3DLock();
    bool hasNewLocation();
    int sendData(const char* data);
    int sendStringAndWait(const std::string &str);
private:
    const char * NMEA_MSG_GXGSA = "GNGSA"; // GSA message (GPGSA, GNGSA etc)
    const char * UNICORE_MSG_PPPNAV = "PPPNAVA"; // Unicore protocol, PPP navigation solution


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
    
    // PppInfo pppInfo;
    uint8_t fixType = 0;      // fix type from GPGSA

    static void logNmeaMessageToSd(const std::string &msg);

    /// default delay between send and receive
    void gpsUartDelay();
};