#include <string>

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


const ble_uuid16_t BleSppServerTask::BLE_SVC_SPP_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_SPP_UUID16_VALUE);
const ble_uuid16_t BleSppServerTask::BLE_SVC_SPP_CHR_UUID16 = BleSppServerTask::buildBleUuid16(BLE_SVC_SPP_CHR_UUID16_VALUE);

const ble_gatt_chr_def BleSppServerTask::spp_characteristics[] = {
    {
        /* Support SPP service */
        .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_CHR_UUID16),
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

const struct ble_gatt_svc_def BleSppServerTask::new_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = reinterpret_cast<const ble_uuid_t *>(&BLE_SVC_SPP_UUID16),
        .includes = nullptr,
        .characteristics = spp_characteristics,
    },
    { }
};


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

void BleSppServerTask::ble_store_config_init()
{

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

    memset(&fields, 0, sizeof fields);

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
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_SVC_SPP_UUID16
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

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

void BleSppServerTask::ble_server_uart_task(void *pvParameters)
{
    MODLOG_DFLT(INFO, "BLE server UART_task started\n");
    int rc = 0;
    for (;;) {
        const uint32_t randomValue = esp_random();
        const std::string asString = "rand value = " + std::to_string(randomValue);
        for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
            /* Check if client has subscribed to notifications */
            if (conn_handle_subs[i]) {
                struct os_mbuf *txom;
                txom = ble_hs_mbuf_from_flat(asString.c_str(), asString.size());
                rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle,
                                                txom);
                if (rc == 0) {
                    MODLOG_DFLT(INFO, "Notification sent successfully");
                } else {
                    MODLOG_DFLT(INFO, "Error in sending notification rc = %d", rc);
                }
            }
        }

        const uint32_t sleepTimeMilliSec = 1 * 1000; 
        vTaskDelay(pdMS_TO_TICKS(sleepTimeMilliSec));
    }

    // uart_event_t event;
    // int rc = 0;
    // for (;;) {
    //     //Waiting for UART event.
    //     if (xQueueReceive(spp_common_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY))            {
    //         switch (event.type) {
    //         //Event of UART receiving data
    //         case UART_DATA:
    //             if (event.size) {
    //                 uint8_t *ntf;
    //                 ntf = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
    //                 memset(ntf, 0x00, event.size);
    //                 uart_read_bytes(UART_NUM_0, ntf, event.size, portMAX_DELAY);

    //                 for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
    //                     /* Check if client has subscribed to notifications */
    //                     if (conn_handle_subs[i]) {
    //                         struct os_mbuf *txom;
    //                         txom = ble_hs_mbuf_from_flat(ntf, event.size);
    //                         rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle,
    //                                                      txom);
    //                         if (rc == 0) {
    //                             MODLOG_DFLT(INFO, "Notification sent successfully");
    //                         } else {
    //                             MODLOG_DFLT(INFO, "Error in sending notification rc = %d", rc);
    //                         }
    //                     }
    //                 }

	// 	    free(ntf);
    //             }
    //             break;
    //         default:
    //             break;
    //         }
    //     }
    // }
    vTaskDelete(nullptr);
}

void BleSppServerTask::ble_spp_uart_init()
{
    // uart_config_t uart_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_RTS,
    //     .rx_flow_ctrl_thresh = 122,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };
    // //Install UART driver, and get the queue.
    // uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_common_uart_queue, 0);
    // //Set UART parameters
    // uart_param_config(UART_NUM_0, &uart_config);
    // //Set UART pins
    // uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //(void *)UART_NUM_0
    xTaskCreate(ble_server_uart_task, "bleSppTask", 4096, nullptr, 8, nullptr);
}


void BleSppServerTask::startServer()
{
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
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    /* Initialize uart driver and start uart task */
    ble_spp_uart_init();

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

#if MYNEWT_VAL(BLE_GATTS)
    int rc;
    /* Register custom service */
    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-ble-spp-svr");
    assert(rc == 0);
#endif

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(ble_spp_server_host_task);
}
