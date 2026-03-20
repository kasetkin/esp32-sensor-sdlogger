#include "gpstask.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <array>


esp_err_t GpsTask::configure()
{
    const uart_config_t uart_config = {
        .baud_rate = UM980_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {}
    };
    // We won't use a buffer for sending data.
    const esp_err_t driverRet = uart_driver_install(GPS_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (driverRet != ESP_OK)
        return driverRet;

    const esp_err_t uartConfigRet = uart_param_config(GPS_UART_PORT, &uart_config);
    if (uartConfigRet != ESP_OK) 
        return uartConfigRet;

    const esp_err_t gpioConfigRet = uart_set_pin(GPS_UART_PORT, UART_TX_GPIO_PIN, UART_RX_GPIO_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (gpioConfigRet != ESP_OK)
        return gpioConfigRet;
    
    return ESP_OK;
}


int GpsTask::sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(GPS_UART_PORT, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

// void GpsTask::tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) {
//         sendData(TX_TASK_TAG, "Hello world");
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

void GpsTask::executeTask()
{
    static const char *RX_TASK_TAG = "RX_TASK";
    const uint32_t readTimeoutInTicks = pdMS_TO_TICKS(1);
    std::array<uint8_t, RX_BUF_SIZE + 1> data;
    while (true) {
        const int rxBytes = uart_read_bytes(GPS_UART_PORT, data.data(), RX_BUF_SIZE, readTimeoutInTicks);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data.data(), rxBytes, ESP_LOG_INFO);
        }
    }
}
