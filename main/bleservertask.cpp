#include <algorithm>
#include <ranges>
#include <string>
#include <mutex>
#include <cmath>
/// ESP-IDF
#include <esp_log.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
/// NimBLE
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <console/console.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#include "bleservertask.h"

uint8_t BleSppServerTask::own_addr_type = 0;
std::array<std::atomic<bool>, CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1> BleSppServerTask::conn_handle_subs;
std::mutex BleSppServerTask::m_dataMutex;
uint16_t BleSppServerTask::ble_battery_read_val_handle = 0;
uint16_t BleSppServerTask::ble_temperature_read_val_handle = 1;
uint16_t BleSppServerTask::ble_humidity_read_val_handle = 2;
uint16_t BleSppServerTask::ble_nmea_read_val_handle = 3;
uint16_t BleSppServerTask::ble_qstarz_read_val_handle = 4;
uint16_t BleSppServerTask::ble_full_log_read_val_handle = 5;

ble_uuid16_t BleSppServerTask::BLE_SVC_BATTERY_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_BATTERY_UUID16_VALUE);
ble_uuid16_t BleSppServerTask::BLE_SVC_ENV_SENSING_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_ENV_SENSING_UUID16_VALUE);
ble_uuid128_t BleSppServerTask::BLE_SVC_SPP_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);

uint16_t BleSppServerTask::ble_tx_write_val_handle = 0;
BleSppServerTask *BleSppServerTask::s_instance = nullptr;

void BleSppServerTask::printBleAddress(const uint8_t value[])
{
    if (value != nullptr)
        MODLOG_DFLT(INFO, "addr = %u%u%u%u%u%u", value[0], value[1], value[2], value[3], value[4], value[5]);

}

ble_uuid16_t BleSppServerTask::buildBleUuid16(const uint16_t value)
{
    return {
        .u = {                          
            .type = BLE_UUID_TYPE_16,   
        },                              
        .value = value
    };
}

int hex2val(char c, uint8_t *value)
{
    if (c >= '0' && c <= '9') {
        *value = c - '0';
    } else if (c >= 'a' && c <= 'f') {
        *value = c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        *value = c - 'A' + 10;
    } else {
        return BLE_HS_EINVAL;
    }
    return 0;
}

ble_uuid128_t BleSppServerTask::buildBleUuid128(const char * str)
{
    MODLOG_DFLT(INFO, "build UUID128 from %s", str);

    /// doesn't work as expected with UUID128 with "0000" in the begining
    /// maybe 'ble_uuid_from_str' is working correctly, but I want full UUID
    // const int err = ble_uuid_from_str(&result, str);

    auto isHexChar = [](char c) static {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    };
    std::string onlyCharsStr = std::string_view(str)
        | std::views::filter(isHexChar)
        | std::ranges::to<std::string>();

    constexpr size_t CORRECT_SIZE = 16 * 2;
    if (onlyCharsStr.size() != 32) {
        MODLOG_DFLT(ERROR, "cannot build UUID from %s", str);
        MODLOG_DFLT(ERROR, "wrong string size after '-' removal, should be %zu, but it is %zu", CORRECT_SIZE, onlyCharsStr.size());
        return {};
    }

    ble_uuid128_t answer {
        .u = {                          
            .type = BLE_UUID_TYPE_128,   
        },
        .value = {},
    };
    for (size_t i = 0; i < 16; i++) {
        uint8_t a = 0;
        uint8_t b = 0;
        const size_t reverseIndex = CORRECT_SIZE - 2 - 2 * i;
        hex2val(onlyCharsStr[reverseIndex], &a);
        hex2val(onlyCharsStr[reverseIndex + 1], &b);
        answer.value[i] = a * 16 + b;
    }

    return answer;
}

/**
 * Logs information about a connection to the console.
 */
void BleSppServerTask::ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    printBleAddress(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    printBleAddress(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    printBleAddress(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    printBleAddress(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
void BleSppServerTask::ble_spp_server_advertise()
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof(fields));

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name;
    name = ble_svc_gap_device_name();
    const auto name_length = strlen(name);

    MODLOG_DFLT(INFO, "advertise BLE device name, length: %zu, name: %s", name_length, name);

    constexpr size_t MAX_NAME_LENGTH = 2;
    if (name_length < MAX_NAME_LENGTH) {
        MODLOG_DFLT(INFO, "name is short, so no magic");
        fields.name = (uint8_t *)name;
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    } else {
        MODLOG_DFLT(INFO, "name is big, so tell full name only as response");
        fields.name = (uint8_t *)name;
        fields.name_len = MAX_NAME_LENGTH;
        fields.name_is_complete = 0;

        struct ble_hs_adv_fields scan_response_fields;
        memset(&scan_response_fields, 0, sizeof scan_response_fields);
        scan_response_fields.name = (uint8_t *)name;
        scan_response_fields.name_len = name_length;
        scan_response_fields.name_is_complete = 1;
        if (const int err = ble_gap_adv_rsp_set_fields(&scan_response_fields); err != 0)
            MODLOG_DFLT(ERROR, "can not setup long name");
    }

    // fields.uuids16 = (ble_uuid16_t[]) {
    //     BLE_SVC_BATTERY_UUID16,
    //     BLE_SVC_ENV_SENSING_UUID16
    // };
    // fields.num_uuids16 = 2;
    // fields.uuids16_is_complete = 1;

    // fields.uuids128 = (ble_uuid128_t[]) {
    //     BleSppServerTask::BLE_SVC_SPP_UUID128
    // };
    // fields.num_uuids128 = 1;
    // fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, ble_spp_server_gap_event, nullptr);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * ble_spp_server uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_server.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
int BleSppServerTask::ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
    /// here can change state of active connections -> 
    /// conn_handle_subs[i] can change, than will modify sender logic
    /// for now try to use std::atomic inside conn_handle_subs

    struct ble_gap_conn_desc desc;
    int rc = 0;
    int connFindRes = 0;
    uint16_t connHandle = CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            connFindRes = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (connFindRes != 0) {
                MODLOG_DFLT(ERROR, "can not find connection hangle");
                return connFindRes;
            }

            // After connection, attempt to switch to 2M PHY
            const uint8_t tx_phy = BLE_GAP_LE_PHY_2M;
            const uint8_t rx_phy = BLE_GAP_LE_PHY_2M;
            rc = ble_gap_set_prefered_le_phy(event->connect.conn_handle, tx_phy, rx_phy, 0);
            if (rc == 0)
                MODLOG_DFLT(INFO, "2M PHY negotiation OK");
            else
                MODLOG_DFLT(WARN, "2M PHY negotiation failed, using 1M");

            ble_spp_server_print_conn_desc(&desc);
        }
        MODLOG_DFLT(INFO, "\n");
        if (event->connect.status != 0 || CONFIG_BT_NIMBLE_MAX_CONNECTIONS > 1) {
            /* Connection failed or if multiple connection allowed; resume advertising. */
            ble_spp_server_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        ble_spp_server_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        connHandle = event->disconnect.conn.conn_handle;
        if (connHandle > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
            MODLOG_DFLT(ERROR, "incorrect connection handle (disconnect) %u", connHandle);
            return ESP_FAIL;
        }

        conn_handle_subs[connHandle] = false;

        /* Connection terminated; resume advertising. */
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        connFindRes = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (connFindRes != 0) {
            MODLOG_DFLT(ERROR, "can not find connection hangle");
            return connFindRes;
        }
        ble_spp_server_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);

        connHandle = event->subscribe.conn_handle;
        if (connHandle > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
            MODLOG_DFLT(ERROR, "incorrect connection handle (subscribe) %u", connHandle);
            return ESP_FAIL;
        }
                            
        conn_handle_subs[connHandle] = true;
        return 0;
    
    /* Encryption change event */
    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        if (event->enc_change.status == 0) {
            MODLOG_DFLT(INFO, "connection encrypted!");
        } else {
            MODLOG_DFLT(ERROR, "connection encryption failed, status: %d",
                     event->enc_change.status);
        }
        return 0;

    /* Repeat pairing event */
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* Delete the old bond */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc != 0) {
            MODLOG_DFLT(ERROR, "failed to find connection, error code %d", rc);
            return rc;
        }
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with pairing operation */
        MODLOG_DFLT(INFO, "repairing...");
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        /* Display action */
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            /* Generate passkey */
            struct ble_sm_io pkey{};
            pkey.action = event->passkey.params.action;
            pkey.passkey = BLE_CONNECTION_KEY;
            MODLOG_DFLT(INFO, "enter passkey %" PRIu32 " on the peer side",
                     pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            if (rc != 0) {
                MODLOG_DFLT(INFO,
                         "failed to inject security manager io, error code: %d",
                         rc);
                return rc;
            }
        }
        return 0;
    default:
        return 0;
    }
}

void BleSppServerTask::ble_spp_server_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

// int BleSppServerTask::ble_store_gen_key(uint8_t key,
//                                 struct ble_store_gen_key *gen_key,
//                                 uint16_t conn_handle)
// {
//     // MODLOG_DFLT(ERROR, "Store gen key\r\n, key = %d, ", key);
//     // return ESP_OK;
// }


void BleSppServerTask::ble_spp_server_on_sync()
{
    int rc;

    const int bleAddrSetupRes = ble_hs_util_ensure_addr(0);
    if (bleAddrSetupRes != 0) {
        MODLOG_DFLT(ERROR, "can not restore BLE MAC address");
        return;
    }

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, nullptr);

    MODLOG_DFLT(INFO, "Device Address: ");
    // printBleAddress(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
    ble_spp_server_advertise();

    if (s_instance)
        s_instance->m_serverIsReady = true;
}

void BleSppServerTask::ble_spp_server_host_task(void *param)
{
    MODLOG_DFLT(INFO, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* Callback function for custom service */
int BleSppServerTask::ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        MODLOG_DFLT(INFO, "Callback for read");
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        MODLOG_DFLT(INFO, "Write event conn=%x attr=%x", conn_handle, attr_handle);
        if (attr_handle == ble_tx_write_val_handle && s_instance
                && s_instance->m_commandReceivedEvent) {
            constexpr uint16_t MAX_CMD = 512;
            const uint16_t pktlen = static_cast<uint16_t>(OS_MBUF_PKTLEN(ctxt->om));
            const uint16_t len = pktlen < MAX_CMD ? pktlen : MAX_CMD;
            char buf[MAX_CMD + 1] = {};
            os_mbuf_copydata(ctxt->om, 0, len, buf);
            s_instance->m_commandReceivedEvent(std::string(buf, len));
        }
        break;

    default:
        MODLOG_DFLT(INFO, "\nDefault Callback");
        break;
    }
    return 0;
}

void BleSppServerTask::gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        MODLOG_DFLT(ERROR, "Incorrect value %u", ctxt->op);
        break;
    }
}

void BleSppServerTask::configureCommandReceivedEvent(CommandReceivedEvent event)
{
    m_commandReceivedEvent = std::move(event);
}

int BleSppServerTask::gatt_svr_init()
{
    BLE_SVC_BATTERY_UUID16 = buildBleUuid16(BLE_SVC_BATTERY_UUID16_VALUE);
    BLE_SVC_ENV_SENSING_UUID16 = buildBleUuid16(BLE_SVC_ENV_SENSING_UUID16_VALUE);
    BLE_SVC_SPP_UUID128 = buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);

    printUuid(BLE_SVC_BATTERY_UUID16);
    printUuid(BLE_SVC_ENV_SENSING_UUID16);
    printUuid(BLE_SVC_SPP_UUID128);

    static const ble_uuid16_t BLE_CHR_BATTERY_LEVEL_UUID16 = buildBleUuid16(BLE_CHR_BATTERY_LEVEL_UUID16_VALUE);
    static const ble_gatt_chr_def battery_characteristics[] = {
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_BATTERY_LEVEL_UUID16),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_battery_read_val_handle,
            .cpfd = nullptr
        },
        { }
    };

    static const ble_uuid16_t BLE_CHR_TEMPERATURE_UUID16 = buildBleUuid16(BLE_CHR_TEMPERATURE_UUID16_VALUE);
    static const ble_uuid16_t BLE_CHR_HUMIDITY_UUID16 = buildBleUuid16(BLE_CHR_HUMIDITY_UUID16_VALUE);
    static const ble_gatt_chr_def env_sensing_characteristics[] = {
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_TEMPERATURE_UUID16),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_temperature_read_val_handle,
            .cpfd = nullptr
        },
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_HUMIDITY_UUID16),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_humidity_read_val_handle,
            .cpfd = nullptr
        },
        { }
    };


    static const ble_uuid128_t BLE_CHR_NMEA_UUID128 = BleSppServerTask::buildBleUuid128(BLE_CHR_NMEA_UUID128_VALUE);
    static const ble_uuid128_t BLE_CHR_QSTARZ_UUID128 = BleSppServerTask::buildBleUuid128(BLE_CHR_QSTARZ_UUID128_VALUE);
    static const ble_uuid128_t BLE_CHR_FULL_LOG_UUID128 = BleSppServerTask::buildBleUuid128(BLE_CHR_FULL_LOG_UUID128_VALUE);
    static const ble_uuid128_t BLE_CHR_TX_UUID128 = BleSppServerTask::buildBleUuid128(BLE_CHR_TX_UUID128_VALUE);

    printUuid(BLE_CHR_NMEA_UUID128);
    printUuid(BLE_CHR_QSTARZ_UUID128);
    printUuid(BLE_CHR_FULL_LOG_UUID128);
    printUuid(BLE_CHR_TX_UUID128);

    s_instance = this;

    static const ble_gatt_chr_def uart_characteristics[] = {
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_NMEA_UUID128),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_nmea_read_val_handle,
            .cpfd = nullptr
        },
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_TX_UUID128),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_tx_write_val_handle,
            .cpfd = nullptr
        },
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_QSTARZ_UUID128),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_qstarz_read_val_handle,
            .cpfd = nullptr
        },
        {
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_CHR_FULL_LOG_UUID128),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC
                   | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
            .min_key_size = 16,
            .val_handle = &ble_full_log_read_val_handle,
            .cpfd = nullptr
        },
        { }
    };

    static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
        {
            /*** Service: BATTERY */
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_BATTERY_UUID16),
            .includes = nullptr,
            .characteristics = battery_characteristics,
        },
        {
            /*** Service: ENV_SENSING */
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_ENV_SENSING_UUID16),
            .includes = nullptr,
            .characteristics = env_sensing_characteristics,
        },
        {
            /*** Service: SPP / UART / QSTARZ / NMEA / LOG */
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            // .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_UUID16),
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_UUID128),
            .includes = nullptr,
            .characteristics = uart_characteristics,
        },
        { }
    };

    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        MODLOG_DFLT(ERROR, "can not ble_gatts_count_cfg(), error %d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "can not ble_gatts_add_svcs(), error %d", rc);
        return rc;
    }

    return 0;
}

void BleSppServerTask::setSensorsValues(const SensorsValues &values)
{
    std::unique_lock writeLock(m_dataMutex);
    m_batteryLevel = values.batteryPercent.has_value()
        ? std::optional<float>{static_cast<float>(values.batteryPercent.value())}
        : std::nullopt;
    m_envTemperature = values.envTemperature;
    m_envHumidity = values.envHumidity;
}

void BleSppServerTask::appendNmea(std::string_view newNmea)
{
    std::unique_lock writeLock(m_dataMutex);
    m_nmeaStream.push_back(std::string(newNmea));
}

void BleSppServerTask::appendLog(std::string_view newLog)
{
    std::unique_lock writeLock(m_dataMutex);
    m_logStream.push_back(std::string(newLog));
}

void BleSppServerTask::transmitQstarzPackets(const std::array<std::vector<std::byte>, 4> &packets)
{
    std::unique_lock readLock(m_dataMutex);

    for (const auto &packet: packets) {
        transmitBuffer(packet.data(), packet.size(), ble_qstarz_read_val_handle);
        vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));
    }
}

inline int BleSppServerTask::bleTx(const void *from, size_t length, uint16_t connHandle, uint16_t valueHandle)
{
    struct os_mbuf *txom;
    txom = ble_hs_mbuf_from_flat(from, length);
    if (txom == nullptr) {
        MODLOG_DFLT(ERROR, "can not allocate buffer, length %zu", length);
        return ESP_FAIL;
    }

    const int rc = ble_gatts_notify_custom(connHandle, valueHandle, txom);
    /// wait 1 millisec, not sure it's necessary 
    // constexpr size_t txDelay = 1; 
    // vTaskDelay(pdMS_TO_TICKS(txDelay));
    return rc;
}

void BleSppServerTask::transmitBuffer(const std::byte *bufferStart, size_t bufferSize, uint16_t value_handle) 
{
    if (bufferSize == 0)
        return;

    MODLOG_DFLT(DEBUG, "transmitLineNow: valueHandle = %d", value_handle);

    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        /* Check if client has subscribed to notifications */
        if (conn_handle_subs[i]) {

            // const size_t attMtu = ble_att_mtu(i);
            const size_t attMtu = 247; /// HARDCODED VALUE, from Interner: The ESP32-C6's controller supports up to 251 bytes PDU, but the GATT layer limits to 247 due to L2CAP overhead. 
            constexpr size_t attHeaderSize = 3; // ATT notification header: 1 opcode + 2 handle
            if (attMtu <= attHeaderSize) {
                MODLOG_DFLT(ERROR, "BLE connection %d with strangely small MTU: %zu. Can not transmitt data", i, attMtu);
                continue;
            }
                
            const size_t maximumPayloadSize = attMtu - attHeaderSize;
            if (maximumPayloadSize == 0) {
                MODLOG_DFLT(ERROR, "code logic error, should be impossible, payloadSize == 0 at connection %d", i);
                continue;
            }

            const size_t fullPacketsCount = bufferSize / maximumPayloadSize;

            for (size_t fullPacketIndex = 0; fullPacketIndex < fullPacketsCount; ++fullPacketIndex) {
                const void *packetStart = bufferStart + fullPacketIndex * maximumPayloadSize;
                const int rc = bleTx(packetStart, maximumPayloadSize, i, value_handle);
                if (rc == 0) {
                    MODLOG_DFLT(DEBUG, "Full Notification (%zu of %zu) sent successfully, size %zu", fullPacketIndex, fullPacketsCount, maximumPayloadSize);
                } else {
                    MODLOG_DFLT(ERROR, "Error in sending full notification (%zu of %zu, size %zu) rc = %d", fullPacketIndex, fullPacketsCount, maximumPayloadSize, rc);
                }
            }

            const bool needSmallPacket = bufferSize % maximumPayloadSize != 0;
            if (needSmallPacket) {
                const void *packetStart = bufferStart + fullPacketsCount * maximumPayloadSize;
                const size_t packetSize = bufferSize % maximumPayloadSize;
                const int rc = bleTx(packetStart, packetSize, i, value_handle);
                if (rc == 0) {
                    MODLOG_DFLT(DEBUG, "The last notification sent successfully, size %zu", packetSize);
                } else {
                    MODLOG_DFLT(ERROR, "Error in sending the last notification (size %zu) rc = %d", packetSize, rc);
                }
            }
        }
    }
}

void BleSppServerTask::transmitBatteryLevel(uint16_t conn_handle)
{
    if (!m_batteryLevel)
        return;

    struct os_mbuf *txom = ble_hs_mbuf_att_pkt();
    if (txom == nullptr)
        return;

    /* Update access buffer value — Battery Level (0x2A19) is uint8, range 0-100% */
    const uint8_t batteryLevelPrepared = static_cast<uint8_t>(
        std::clamp(std::round(m_batteryLevel.value()), 0.0f, 100.0f));
    const int rc1 = os_mbuf_append(txom, &batteryLevelPrepared, sizeof(batteryLevelPrepared));

    if (rc1 != 0) {
        /// not shure if this is correct way to free
        if (txom)
            os_mbuf_free(txom);
            
        return;
    }

    const int rc2 = ble_gatts_notify_custom(conn_handle, ble_battery_read_val_handle, txom);
    if (rc2 == 0) {
        MODLOG_DFLT(DEBUG, "Notification sent successfully: battery");
    } else {
        MODLOG_DFLT(ERROR, "Error in sending notification rc = %d", rc2);
    }
}

void BleSppServerTask::transmitEnvHumidity(uint16_t conn_handle)
{
    if (!m_envHumidity)
        return;

    struct os_mbuf *txom = ble_hs_mbuf_att_pkt();
    if (txom == nullptr)
        return;

    /* Update access buffer value */
    const int16_t humidityPreparedSigned = static_cast<int16_t>(std::round(m_envHumidity.value() * 100.0));
    const uint16_t humidityPrepared = static_cast<uint16_t>(humidityPreparedSigned);
    static uint8_t env_humidity_chr_val[2] = {0, 0};
    env_humidity_chr_val[1] = humidityPrepared / 256;
    env_humidity_chr_val[0] = humidityPrepared % 256;
    const int rc1 = os_mbuf_append(txom, &env_humidity_chr_val, sizeof(env_humidity_chr_val));

    if (rc1 != 0) {
        /// not shure if this is correct way to free
        if (txom)
            os_mbuf_free(txom);
            
        return;
    }

    const int rc2 = ble_gatts_notify_custom(conn_handle, ble_humidity_read_val_handle, txom);
    if (rc2 == 0) {
        MODLOG_DFLT(DEBUG, "Notification sent successfully: humidity");
    } else {
        MODLOG_DFLT(ERROR, "Error in sending notification rc = %d", rc2);
    }
}

void BleSppServerTask::transmitEnvTemperature(uint16_t conn_handle)
{
    if (!m_envTemperature)
        return;

    MODLOG_DFLT(DEBUG, "BLE: transmit temperature value %f", m_envTemperature.value());
    struct os_mbuf *txom = ble_hs_mbuf_att_pkt();
    if (txom == nullptr) {
        MODLOG_DFLT(ERROR, "BLE: transmit temperature, buffer allocation - ERROR");
        return;
    }

    MODLOG_DFLT(DEBUG, "BLE: transmit temperature, buffer allocation - ok");
    /* Update access buffer value */
    const int16_t temperaturePreparedSigned = static_cast<int16_t>(std::round(m_envTemperature.value() * 100.0));
    const uint16_t temperaturePrepared = static_cast<uint16_t>(temperaturePreparedSigned); /// prepare for bytes operations
    static uint8_t env_temperature_chr_val[2] = {0, 0};
    env_temperature_chr_val[1] = temperaturePrepared / 256;
    env_temperature_chr_val[0] = temperaturePrepared % 256;
    const int rc1 = os_mbuf_append(txom, &env_temperature_chr_val, sizeof(env_temperature_chr_val));

    if (rc1 != 0) {
        MODLOG_DFLT(ERROR, "BLE: transmit temperature, os_mbuf_append rc = %d", rc1);
        /// not shure if this is correct way to free
        if (txom)
            os_mbuf_free(txom);
            
        return;
    }

    MODLOG_DFLT(DEBUG, "BLE: transmit temperature, before 'ble_gatts_notify_custom'");
    const int rc2 = ble_gatts_notify_custom(conn_handle, ble_temperature_read_val_handle, txom);
    if (rc2 == 0) {
        MODLOG_DFLT(DEBUG, "BLE: transmit temperature, Notification sent successfully: temperature");
    } else {
        MODLOG_DFLT(ERROR, "BLE: transmit temperature, Error in sending notification rc = %d", rc2);
    }
}

void BleSppServerTask::sendAllData()
{
    if (!m_serverIsReady)
        return;

    std::unique_lock readLock(m_dataMutex);

    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        if (conn_handle_subs[i]) {
            transmitBatteryLevel(i);
            transmitEnvTemperature(i);
            transmitEnvHumidity(i);
        }
    }
    
    {
        for (const auto &line : m_logStream)
            transmitBuffer(reinterpret_cast<const std::byte *>(line.c_str()), line.size(), ble_full_log_read_val_handle);

        m_logStream.clear();
    }

    {
        for (const auto &line : m_nmeaStream)
            transmitBuffer(reinterpret_cast<const std::byte *>(line.c_str()), line.size(), ble_nmea_read_val_handle);

        m_nmeaStream.clear();
    }
}

void BleSppServerTask::bleSenderTask()
{
    MODLOG_DFLT(INFO, "BLE server DataSender started\n");
    while (!m_terminateASAP) {
        sendAllData();
        const uint32_t sleepTimeMilliSec = 1000; // 1 sec
        vTaskDelay(pdMS_TO_TICKS(sleepTimeMilliSec));
    }
}

void BleSppServerTask::dataSenderTaskInit()
{
    xTaskCreate([](void *bleTask)
    { 
        auto asObject = reinterpret_cast<BleSppServerTask *>(bleTask);
        asObject->bleSenderTask();
        vTaskDelete(nullptr);
    }, "bleSppTask", 16384, this, 6, nullptr);
}

void BleSppServerTask::terminate()
{
    m_terminateASAP = true;
    nimble_port_stop();
    /// what else?
};

void BleSppServerTask::startServer()
{
    BleSppServerTask::BLE_SVC_SPP_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);
    printUuid(BleSppServerTask::BLE_SVC_SPP_UUID128);

    /// already done in main.cpp
    // esp_err_t ret = nvs_flash_init();

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }

    /* Initialize connection_handle array */
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++)
        conn_handle_subs[i] = false;

    /* Initialize uart driver and start uart task */
    dataSenderTaskInit();

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    // ble_hs_cfg.store_gen_key_cb = ble_store_gen_key;

    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_sc_only = 1;
    ble_hs_cfg.sm_sec_lvl = 4;

    /* Register custom service */
    const int gattInitRes = gatt_svr_init();
    if (gattInitRes != 0) {
        MODLOG_DFLT(ERROR, "can not inin GATT server");
        return;
    }

    /* Set the default device name. */
    MODLOG_DFLT(INFO, "pre-setup BLE device name");
    const int gapNameSetRes = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (gapNameSetRes != 0) {
        MODLOG_DFLT(ERROR, "can not setup GAP name");
        return;
    }

    /// maximum possible MTU
    const int mtuSetupRes = ble_att_set_preferred_mtu(BLE_ATT_MTU_MAX);
    if (mtuSetupRes != 0) {
        MODLOG_DFLT(ERROR, "can not setup MTU to %d", BLE_ATT_MTU_MAX);
        return;
    }

    nimble_port_freertos_init(ble_spp_server_host_task);
    /// m_serverIsReady is set in ble_spp_server_on_sync() after the host stack synchronizes
}

void BleSppServerTask::printUuid(const ble_uuid16_t &uuid)
{
    char buf[BLE_UUID_STR_LEN];
    const char * uuidAsStr = ble_uuid_to_str(reinterpret_cast<const ble_uuid_t *>(&uuid), buf);
    MODLOG_DFLT(INFO, "uuid: type = %u, value = %s", uuid.u, uuidAsStr);
}

void BleSppServerTask::printUuid(const ble_uuid128_t &uuid)
{
    char buf[BLE_UUID_STR_LEN];
    const char * uuidAsStr = ble_uuid_to_str(reinterpret_cast<const ble_uuid_t *>(&uuid), buf);
    MODLOG_DFLT(INFO, "uuid: type = %u, value = %s", uuid.u, uuidAsStr);
}