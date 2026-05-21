#pragma once

#define STATIC_PASSKEY

#include <stdbool.h>
#include <functional>
#include <mutex>
#include <atomic>
#include <array>
#include <vector>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <algorithm>
#include <iterator>
#include <span>
#include <host/ble_hs.h>
#include <nimble/ble.h>
#include <nimble/nimble_port_freertos.h>
#include "sensorstask.h"

class UuidTools
{
public:
    static consteval ble_uuid16_t buildBleUuid16(const uint16_t value)
    {
        return {
            .u = { .type = BLE_UUID_TYPE_16 },
            .value = value,
        };
    }

    static consteval ble_uuid128_t buildBleUuid128(std::string_view str)
    {
        constexpr size_t HEX_COUNT = 32;
        char hex[HEX_COUNT] = {};
        auto isHexChar = [](char c) static {
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        };
        std::ranges::copy_if(str, std::counted_iterator(std::begin(hex), HEX_COUNT), isHexChar);

        ble_uuid128_t answer = {
            .u = { .type = BLE_UUID_TYPE_128 },
            .value = {},
        };

        auto hexToInt = [](char c) static -> uint8_t {
            if (c >= '0' && c <= '9')
                return static_cast<uint8_t>(c - '0');
            if (c >= 'a' && c <= 'f')
                return static_cast<uint8_t>(c - 'a' + 10);
            if (c >= 'A' && c <= 'F')
                return static_cast<uint8_t>(c - 'A' + 10);

            return 0;
        };

        for (size_t i = 0; i < 16; ++i) {
            const size_t ri = HEX_COUNT - 2 - 2 * i;
            answer.value[i] = static_cast<uint8_t>(hexToInt(hex[ri]) * 16u + hexToInt(hex[ri + 1]));
        }
        return answer;
    }
};

class BleSppServerTask
{
public:
    void startServer();
    void terminate();

    void setSensorsValues(const SensorsValues &values);

    void appendNmea(std::string_view newNmea);
    void appendLog(std::string_view newLog);
    void transmitQstarzPackets(const std::array<std::vector<std::byte>, 4> &packets);

    using CommandReceivedEvent = std::function<void(std::string_view command)>;
    void configureCommandReceivedEvent(CommandReceivedEvent event);

private:
    static constexpr uint32_t BLE_CONNECTION_KEY = 654321;
    static uint8_t own_addr_type;
    static std::array<std::atomic<bool>, CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1> conn_handle_subs;
    static std::mutex m_dataMutex; /// protect connections states and data

    static uint16_t ble_battery_read_val_handle;
    static uint16_t ble_temperature_read_val_handle;
    static uint16_t ble_humidity_read_val_handle;
    static uint16_t ble_nmea_read_val_handle;
    static uint16_t ble_qstarz_read_val_handle;
    static uint16_t ble_full_log_read_val_handle;
    static uint16_t ble_tx_write_val_handle;
    static BleSppServerTask *s_instance;

    static constexpr uint32_t TX_DELAY_MS = 10;

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
    static constexpr char BLE_CHR_NMEA_UUID128_VALUE[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";        // NUS TX: peripheral notifies central — NMEA/GNSS output
    static constexpr char BLE_CHR_TX_UUID128_VALUE[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";          // NUS RX: central writes to peripheral — UART input (send to UM980) and custom cmds
    static constexpr char BLE_CHR_QSTARZ_UUID128_VALUE[] = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E";      // from "qstarz" racing gps, binary format, 4 packets (20 + 20 + 20 + 4) bytes
    static constexpr char BLE_CHR_FULL_LOG_UUID128_VALUE[] = "6E400005-B5A3-F393-E0A9-E50E24DCCA9E";    // full log, same as SD card "xxx.log" file

    std::optional<float> m_batteryLevel;
    std::optional<float> m_envTemperature;
    std::optional<float> m_envHumidity;
    std::vector<std::string> m_nmeaStream;
    std::vector<std::string> m_logStream;
    CommandReceivedEvent m_commandReceivedEvent;
    std::atomic<bool> m_serverIsReady{false};
    std::atomic<bool> m_terminateASAP{false};

    static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
    static void ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc);
    static void ble_spp_server_advertise();
    static void ble_spp_server_on_reset(int reason);
    static void ble_spp_server_on_sync();
    static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
    static void ble_spp_server_host_task(void *param); /// should be static or lambda

    static constexpr ble_uuid16_t BLE_SVC_BATTERY_UUID16 = UuidTools::buildBleUuid16(BLE_SVC_BATTERY_UUID16_VALUE);
    static constexpr ble_uuid16_t BLE_SVC_ENV_SENSING_UUID16 = UuidTools::buildBleUuid16(BLE_SVC_ENV_SENSING_UUID16_VALUE);
    static constexpr ble_uuid128_t BLE_SVC_SPP_UUID128      = UuidTools::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);
    static constexpr ble_uuid128_t BLE_CHR_NMEA_UUID128     = UuidTools::buildBleUuid128(BLE_CHR_NMEA_UUID128_VALUE);
    static constexpr ble_uuid128_t BLE_CHR_TX_UUID128       = UuidTools::buildBleUuid128(BLE_CHR_TX_UUID128_VALUE);
    static constexpr ble_uuid128_t BLE_CHR_QSTARZ_UUID128   = UuidTools::buildBleUuid128(BLE_CHR_QSTARZ_UUID128_VALUE);
    static constexpr ble_uuid128_t BLE_CHR_FULL_LOG_UUID128 = UuidTools::buildBleUuid128(BLE_CHR_FULL_LOG_UUID128_VALUE);


    static void printUuid(const ble_uuid16_t &uuid);
    static void printUuid(const ble_uuid128_t &uuid);
    static void printBleAddress(const uint8_t value[]);
    // static int ble_store_gen_key(uint8_t key,
    //                              struct ble_store_gen_key *gen_key,
    //                              uint16_t conn_handle);

    /// @brief register GATT services
    [[nodiscard("BLE services not registered on failure")]]
    int gatt_svr_init();

    void dataSenderTaskInit();
    void bleSenderTask();
    void sendAllData();
    inline int bleTx(std::span<const std::byte> data, uint16_t connHandle, uint16_t valueHandle);
    void transmitBuffer(std::span<const std::byte> buffer, uint16_t value_handle);
    void transmitEnvHumidity(uint16_t conn_handle);
    void transmitEnvTemperature(uint16_t conn_handle);
    void transmitBatteryLevel(uint16_t conn_handle);
};
    
