#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "TinyGPSPlus.h"

class GpsTask
{
public:
    static constexpr int UART_TX_GPIO_PIN = GPIO_NUM_16;
    static constexpr int UART_RX_GPIO_PIN = GPIO_NUM_17;
    static constexpr int UM980_UART_BAUDRATE = 115200;
    static constexpr int RX_BUF_SIZE = 1024;
    static constexpr uart_port_t GPS_UART_PORT = UART_NUM_1;

    /// init UART and structures
    esp_err_t configure();
    
    void executeTask();

    int sendData(const char* logName, const char* data);
private:
    TinyGPSPlus m_gps;

};