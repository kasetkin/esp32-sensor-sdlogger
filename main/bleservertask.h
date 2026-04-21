#pragma once

#define STATIC_PASSKEY

#include <stdbool.h>
#include <mutex>
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

    void appendData(const std::string &newData);
    void transmitLineNow(const std::string &line);
private:
    static constexpr uint32_t BLE_CONNECTION_KEY = 654321;
    static uint8_t own_addr_type;
    static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
    static uint16_t ble_spp_svc_gatt_read_val_handle;

    static constexpr char BLE_DEVICE_NAME[] = "QSTARZ_EMULATOR"; // so 'Bluetooth GNSS' app will try to connect

    /// Nordic semiconductors
    static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"; // "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; , for easy Ctrl+F
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // default RT/TX from NordicSemiCond, "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"; // from "qstarz" racing gps, some custom JSON
    
    /// CC254X  --  this pair of UUIDs works with 'Serial Bluetooth Terminal' app 
    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "0000ffe0-0000-1000-8000-00805F9B34FB";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "0000ffe1-0000-1000-8000-00805F9B34FB";

    // public static final UUID nordic_uart_service_uuid = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "00001101-0000-1000-8000-00805F9B34FB";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "00002902-0000-1000-8000-00805f9b34fb";

    static ble_uuid128_t BLE_SVC_SPP_UUID128;

    // static ble_uuid128_t BLE_SVC_SPP_CHR_UUID128;
    // static constexpr char BLE_SVC_SPP_UUID128_VALUE[] = "F000C0E0-0451-4000-B000-000000000000";
    // static constexpr char BLE_SVC_SPP_CHR_UUID128_VALUE[] = "F000C0E1-0451-4000-B000-000000000000";
    // static const ble_uuid128_t BLE_SVC_SPP_CHR_UUID128;
    // static const ble_gatt_chr_def spp_characteristics[];
    // static const ble_gatt_svc_def new_ble_svc_gatt_defs[];


    mutable std::mutex m_dataMutex;
    mutable std::mutex m_dataTxMutex;
    std::vector<std::string> m_data;

    static void print_addr(const uint8_t value[]);
    static ble_uuid16_t buildBleUuid16(const uint16_t value);
    static ble_uuid128_t buildBleUuid128(const char * str);
    static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
    static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
    static void ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc);
    static void ble_spp_server_advertise();
    static void ble_spp_server_on_reset(int reason);
    static void ble_spp_server_on_sync();
    // static int ble_store_gen_key(uint8_t key,
    //                              struct ble_store_gen_key *gen_key,
    //                              uint16_t conn_handle);
    static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
    static void ble_spp_server_host_task(void *param); /// should be static or lambda
    static void printUuid128(const ble_uuid128_t &uuid);

    int gatt_svr_init();
    void dataSenderTaskInit();
    void bleSenderTask();
    void sendAllData();
};
    
