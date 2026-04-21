#include <string>
#include <mutex>

#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "bleservertask.h"

uint8_t BleSppServerTask::own_addr_type = 0;
bool BleSppServerTask::conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
uint16_t BleSppServerTask::ble_spp_svc_gatt_read_val_handle = 0;


// const ble_uuid16_t BleSppServerTask::BLE_SVC_SPP_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_SPP_UUID16_VALUE);
// const ble_uuid16_t BleSppServerTask::BLE_SVC_SPP_CHR_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_SPP_CHR_UUID16_VALUE);
ble_uuid128_t BleSppServerTask::BLE_SVC_SPP_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);
// ble_uuid128_t BleSppServerTask::BLE_SVC_SPP_CHR_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_CHR_UUID128_VALUE);

void BleSppServerTask::print_addr(const uint8_t value[])
{
    if (value != nullptr)
        MODLOG_DFLT(INFO, "addr = %d%d%d%d%d%d", value[0], value[1], value[2], value[3], value[4], value[5]);
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
    ble_uuid_any_t result{};
    MODLOG_DFLT(INFO, "build UUID128 from %s", str);

    /// doesn't work as expected with UUID128 with "0000" in the begining
    /// maybe 'ble_uuid_from_str' is working correctly, but I want full UUID
    // const int err = ble_uuid_from_str(&result, str);

    std::string onlyCharsStr;
    const size_t inputLenght = strlen(str);
    for (size_t i = 0; i < inputLenght; ++i) {
        char x = str[i];
        uint8_t charAsValue = 0;
        const int convertResult = hex2val(x, &charAsValue);
        if (convertResult == 0)
            onlyCharsStr += x;
    }

    constexpr size_t CORRECT_SIZE = 16 * 2;
    if (onlyCharsStr.size() != 32) {
        MODLOG_DFLT(ERROR, "cannot build UUID from %s", str);
        MODLOG_DFLT(ERROR, "wrong string size after '-' removal, should be %d, but it is %d", CORRECT_SIZE, onlyCharsStr.size());
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

    result.u128 = answer;
    return answer;
}

/**
 * Logs information about a connection to the console.
 */
void BleSppServerTask::ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
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
                   BLE_HS_ADV_F_BREDR_UNSUP |
                   BLE_HS_ADV_F_DISC_LTD;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name;
    name = ble_svc_gap_device_name();
    const auto name_length = strlen(name);

    MODLOG_DFLT(INFO, "advertise BLE device name, length: %u, name: %s", name_length, name);

    if (name_length < 5) {
        MODLOG_DFLT(INFO, "name is short, so no magic");
        fields.name = (uint8_t *)name;
        fields.name_len = strlen(name);
        fields.name_is_complete = 1;
    } else {
        MODLOG_DFLT(INFO, "name is big, so tell full name only as response");
        fields.name = (uint8_t *)name;
        fields.name_len = 5;
        fields.name_is_complete = 0;

        struct ble_hs_adv_fields scan_response_fields;
        memset(&scan_response_fields, 0, sizeof scan_response_fields);
        scan_response_fields.name = (uint8_t *)name;
        scan_response_fields.name_len = name_length;
        scan_response_fields.name_is_complete = 1;
        const int err = ble_gap_adv_rsp_set_fields(&scan_response_fields);
        if (err != 0) {
            MODLOG_DFLT(ERROR, "can not setup long name");
        }
    }

    // fields.uuids16 = (ble_uuid16_t[]) {
    //     BLE_SVC_SPP_UUID16
    // };
    // fields.num_uuids16 = 1;
    // fields.uuids16_is_complete = 1;

    fields.uuids128 = (ble_uuid128_t[]) {
        BleSppServerTask::BLE_SVC_SPP_UUID128
    };
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

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
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
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

        conn_handle_subs[event->disconnect.conn.conn_handle] = false;

        /* Connection terminated; resume advertising. */
        ble_spp_server_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
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
        conn_handle_subs[event->subscribe.conn_handle] = true;
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

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

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
    // print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
    ble_spp_server_advertise();
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
        MODLOG_DFLT(INFO, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
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
        break;
    }
}

int BleSppServerTask::gatt_svr_init()
{
    BLE_SVC_SPP_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);
    printUuid128(BLE_SVC_SPP_UUID128);

    static const ble_uuid128_t BLE_SVC_SPP_CHR_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_CHR_UUID128_VALUE);
    printUuid128(BLE_SVC_SPP_CHR_UUID128);

    static const ble_gatt_chr_def spp_characteristics[] = {
        {
            /* Support SPP service */
            // .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_CHR_UUID16),
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_CHR_UUID128),
            .access_cb = ble_svc_gatt_handler,
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            .min_key_size = 0,
            .val_handle = &ble_spp_svc_gatt_read_val_handle,
            .cpfd = nullptr
        },
        { }
    };

    static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
        {
            /*** Service: SPP */
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            // .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_UUID16),
            .uuid = reinterpret_cast<const ble_uuid_t *>(&BleSppServerTask::BLE_SVC_SPP_UUID128),
            .includes = nullptr,
            .characteristics = spp_characteristics,
        },
        { }
    };

    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void BleSppServerTask::transmitLineNow(const std::string &line) 
{
    std::unique_lock txLock(m_dataTxMutex);
    // MODLOG_DFLT(INFO, "new NMEA line is: %s", line.c_str());
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        /* Check if client has subscribed to notifications */
        if (conn_handle_subs[i]) {
            struct os_mbuf *txom;
            txom = ble_hs_mbuf_from_flat(line.c_str(), line.size());
            const int rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle,
                                            txom);
            if (rc == 0) {
                MODLOG_DFLT(INFO, "Notification sent successfully");
            } else {
                MODLOG_DFLT(INFO, "Error in sending notification rc = %d", rc);
            }
        }
    }
}

void BleSppServerTask::sendAllData()
{
    std::unique_lock readLock(m_dataMutex);
    for (const auto &dataLine : m_data)
        transmitLineNow(dataLine);

    m_data.clear();
}

void BleSppServerTask::bleSenderTask()
{
    MODLOG_DFLT(INFO, "BLE server DataSender started\n");
    for (;;) {
        sendAllData();
        const uint32_t sleepTimeMilliSec = 1000; // 0.1 sec
        vTaskDelay(pdMS_TO_TICKS(sleepTimeMilliSec));
    }

    vTaskDelete(nullptr);
}

void BleSppServerTask::dataSenderTaskInit()
{
    xTaskCreate([](void *bleTask)
    { 
        auto asObject = reinterpret_cast<BleSppServerTask *>(bleTask);
        asObject->bleSenderTask();
    }, "bleSppTask", 4096, this, 8, nullptr);
}

void BleSppServerTask::appendData(const std::string &newData)
{
    std::unique_lock writeLock(m_dataMutex);
    m_data.push_back(newData); //copy
}

void BleSppServerTask::startServer()
{
    BleSppServerTask::BLE_SVC_SPP_UUID128 = BleSppServerTask::buildBleUuid128(BLE_SVC_SPP_UUID128_VALUE);
    printUuid128(BleSppServerTask::BLE_SVC_SPP_UUID128);

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
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
    // 0 -- disconnect 531 after 5 seconds
    // 1 -- doesn't bond
    // 2 -- disconnect 531 after 5 seconds
    // 3 -- disconnect 531 after 5 seconds
    // 4 -- disconnect 531 after 5 seconds
    ble_hs_cfg.sm_sec_lvl = 4;

    int rc;
    /* Register custom service */
    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    MODLOG_DFLT(INFO, "pre-setup BLE device name");
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    assert(rc == 0);

    nimble_port_freertos_init(ble_spp_server_host_task);
}

void BleSppServerTask::printUuid128(const ble_uuid128_t &uuid)
{
    char buf[BLE_UUID_STR_LEN];
    const char * uuidAsStr = ble_uuid_to_str(reinterpret_cast<const ble_uuid_t *>(&uuid), buf);
    MODLOG_DFLT(INFO, "uuid: type = %d, value = %s", uuid.u, uuidAsStr);
}