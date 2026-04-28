#pragma once

#define STATIC_PASSKEY

#include <stdbool.h>
#include <mutex>
#include <array>
#include <vector>
#include <string>
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "nimble/nimble_port_freertos.h"


/* Define new custom service */

class BleSppServerTask
{
public:
    void startServer();

    void setBatteryLevel(float level);
    void setEnvHumidity(float humidity);
    void setEnvTemperature(float temperature);

    void appendNmea(const std::string &newNmea);
    void appendLog(const std::string &newNmea);
    void transmitQstarzPackets(const std::array<std::string, 4> &packets);

private:
    static constexpr uint32_t BLE_CONNECTION_KEY = 654321;
    static uint8_t own_addr_type;
    static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

    static uint16_t ble_battery_read_val_handle;
    static uint16_t ble_temperature_read_val_handle;
    static uint16_t ble_humidity_read_val_handle;
    static uint16_t ble_nmea_read_val_handle;
    static uint16_t ble_qstarz_read_val_handle;
    static uint16_t ble_full_log_read_val_handle;

    static constexpr uint32_t TX_DELAY_MICROSEC = 10;

    static constexpr char BLE_DEVICE_NAME[] = "QSTARZ_EMULATOR"; // so 'Bluetooth GNSS' app will try to connect

    /// Battery service
    static constexpr uint16_t BLE_SVC_BATTERY_UUID16_VALUE = 0x180Fu;
    static constexpr uint16_t BLE_CHR_BATTERY_LEVEL_UUID16_VALUE = 0x2A19u;

    /// Environment sensing service
    static constexpr uint16_t BLE_SVC_ENV_SENSING_UUID16_VALUE = 0x181Au;
    static constexpr uint16_t BLE_CHR_TEMPERATURE_UUID16_VALUE = 0x2A6Eu;
    static constexpr uint16_t BLE_CHR_HUMIDITY_UUID16_VALUE = 0x2A6Fu;

    /// Nordic semiconductors == UART SPP
    /// some info can be found here: https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html
    static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";         // NordicSemiCond value for UART (SPP mode in classic bluetooth)
    static constexpr char BLE_CHR_NMEA_UUID128_VALUE[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";        // NMEA (and more) output from GNSS module (default UART RX from NordicSemiCond)
    
    /// \todo implement sending commands from BLE to UM980 module
    static constexpr char BLE_CHR_TX_UUID128_VALUE[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
    
    static constexpr char BLE_CHR_QSTARZ_UUID128_VALUE[] = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E";      // from "qstarz" racing gps, binary format, 4 packets (20 + 20 + 20 + 4) bytes
    static constexpr char BLE_CHR_FULL_LOG_UUID128_VALUE[] = "6E400005-B5A3-F393-E0A9-E50E24DCCA9E";    // full log, same as SD card "xxx.log" file
    
    /// CC254X  --  this pair of UUIDs works with 'Serial Bluetooth Terminal' app 
    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "0000ffe0-0000-1000-8000-00805F9B34FB";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "0000ffe1-0000-1000-8000-00805F9B34FB";

    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "00001101-0000-1000-8000-00805F9B34FB";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "00002902-0000-1000-8000-00805f9b34fb";

    static ble_uuid16_t BLE_SVC_BATTERY_UUID16;
    static ble_uuid16_t BLE_SVC_ENV_SENSING_UUID16;
    static ble_uuid128_t BLE_SVC_SPP_UUID128;

    // static ble_uuid128_t BLE_SVC_SPP_CHR_UUID128;
    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "F000C0E0-0451-4000-B000-000000000000";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "F000C0E1-0451-4000-B000-000000000000";
    // static const ble_uuid128_t BLE_SVC_SPP_CHR_UUID128;
    // static const ble_gatt_chr_def spp_characteristics[];
    // static const ble_gatt_svc_def new_ble_svc_gatt_defs[];


    mutable std::mutex m_dataMutex;
    mutable std::mutex m_dataTxMutex;
    float m_batteryLevel = -1.0;
    float m_envTemperature = -275.0;
    float m_envHumidity = -1.0;
    std::vector<std::string> m_nmeaStream;
    std::vector<std::string> m_logStream;

    static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
    static void ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc);
    static void ble_spp_server_advertise();
    static void ble_spp_server_on_reset(int reason);
    static void ble_spp_server_on_sync();
    static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
    static void ble_spp_server_host_task(void *param); /// should be static or lambda

    static ble_uuid16_t buildBleUuid16(const uint16_t value);
    static ble_uuid128_t buildBleUuid128(const char * str);
    static void printUuid(const ble_uuid16_t &uuid);
    static void printUuid(const ble_uuid128_t &uuid);
    static void printBleAddress(const uint8_t value[]);
    // static int ble_store_gen_key(uint8_t key,
    //                              struct ble_store_gen_key *gen_key,
    //                              uint16_t conn_handle);

    /// @brief register GATT services
    int gatt_svr_init();

    void dataSenderTaskInit();
    void bleSenderTask();
    void sendAllData();
    void transmitLineNow(const std::string &line, uint16_t value_handle);
    void transmitEnvHumidity(uint16_t conn_handle);
    void transmitEnvTemperature(uint16_t conn_handle);
    void transmitBatteryLevel(uint16_t conn_handle);
};
    
