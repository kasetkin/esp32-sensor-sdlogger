#pragma once

#define STATIC_PASSKEY

#include <stdbool.h>
#include "host/ble_hs.h"
#include "modlog/modlog.h"
#include "nimble/ble.h"
#include "nimble/nimble_port_freertos.h"


/* Define new custom service */

class BleSppServerTask
{
public:
    void startServer();

private:
    static constexpr uint32_t BLE_CONNECTION_KEY = 654321;
    static uint8_t own_addr_type;
    static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
    static uint16_t ble_spp_svc_gatt_read_val_handle;

    /* 16 Bit SPP Service UUID */
    static constexpr uint16_t BLE_SVC_SPP_UUID16_VALUE = 0xABF0;
    static const ble_uuid16_t BLE_SVC_SPP_UUID16;

    /* 16 Bit SPP Service Characteristic UUID */
    static constexpr uint16_t BLE_SVC_SPP_CHR_UUID16_VALUE = 0xABF1;
    static const ble_uuid16_t BLE_SVC_SPP_CHR_UUID16;

    static const ble_gatt_chr_def spp_characteristics[];
    static const ble_gatt_svc_def new_ble_svc_gatt_defs[];


    QueueHandle_t spp_common_uart_queue = nullptr;

    void ble_store_config_init();
    
    static void print_addr(const uint8_t value[]);
    static ble_uuid16_t buildBleUuid16(const uint16_t value);
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
    int gatt_svr_init();
    static void ble_server_uart_task(void *pvParameters);
    void ble_spp_uart_init();
};
