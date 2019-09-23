/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdk_config.h"

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_scan.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_delay.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_ble_ctrl.h"

// ble common
cBle_ctrl_on_ble_evt_handler_func m_ble_evt_cb =NULL;


//ble stack
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

bool ble_connect_on = false;

//ble advertising
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
cBle_ctrl_on_adv_evt_handler_func m_ble_on_adv_evt_cb = NULL;
bool m_ble_advertising_req_state = false;
bool m_ble_advertising_state = false;

//ble gap_params
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

//ble conn_params
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */

//ble gatt
NRF_BLE_GATT_DEF(m_gatt);
static uint16_t   m_ble_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//ble dfu


//ble services


//ble nus
#define BLE_NUS_MAX_CLIENT              1
#if(BLE_NUS_MAX_CLIENT > NRF_SDH_BLE_TOTAL_LINK_COUNT)
#error "not support!"
#endif

BLE_NUS_DEF(m_nus, BLE_NUS_MAX_CLIENT);
cBle_ctrl_nus_recv_data_handler_func m_ble_on_nus_data_cb;
static void nus_data_handler(ble_nus_evt_t * p_evt);


//use peer manager
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */
static bool m_ble_bonds_success_flag = false;

//use ble scan
#define SCAN_INTERVAL           0x00A0                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */
bool m_ble_scan_run_flag = false;
cBle_ctrl_scan_recv_handler_func m_ble_scan_cb;

//use ble nus
/******************************************************************************/
// ble common
/******************************************************************************/
void cfg_ble_get_ble_mac_address(uint8_t *p_buf /*6byte*/)
{
    uint32_t err_code;
    ble_gap_addr_t      addr;

    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    p_buf[0] = addr.addr[5];
    p_buf[1] = addr.addr[4];
    p_buf[2] = addr.addr[3];
    p_buf[3] = addr.addr[2];
    p_buf[4] = addr.addr[1];
    p_buf[5] = addr.addr[0];
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            cPrintLog(CDBG_BLE_DBG, "Fast advertising.\n");
            m_ble_advertising_state = true;
            if(m_ble_on_adv_evt_cb)m_ble_on_adv_evt_cb(cBle_ADV_START);
            break;

        case BLE_ADV_EVT_IDLE:
            cPrintLog(CDBG_BLE_DBG, "advertising to idle\n");
            m_ble_advertising_state = false;
            m_ble_advertising_req_state = false;
            if(m_ble_on_adv_evt_cb)m_ble_on_adv_evt_cb(cBle_ADV_STOP);
            break;

        default:
            break;
    }
}

//ble stack
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            cPrintLog(CDBG_BLE_INFO, "BLE_GAP_EVT_DISCONNECTED!\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ble_connect_on = false;
            if(m_ble_evt_cb)m_ble_evt_cb(cBle_GAP_DISCONNECTED, NULL);
            break;

        case BLE_GAP_EVT_CONNECTED:
            cPrintLog(CDBG_BLE_INFO, "BLE_GAP_EVT_CONNECTED!\n");
            m_ble_advertising_state = false;
            if(m_ble_on_adv_evt_cb)m_ble_on_adv_evt_cb(cBle_ADV_STOP);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            ble_connect_on = true;
            if(m_ble_evt_cb)m_ble_evt_cb(cBle_GAP_CONNECTED, NULL);
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            cPrintLog(CDBG_BLE_INFO, "PHY update request.\n");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            cPrintLog(CDBG_BLE_INFO, "GATT Client Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            cPrintLog(CDBG_BLE_INFO, "GATT Server Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        case BLE_GAP_EVT_ADV_REPORT:
            {
                const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
                const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
                if(m_ble_scan_cb)m_ble_scan_cb(p_adv_report);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            {
                const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
                {
                    m_ble_scan_run_flag = false;
                }
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

void cfg_ble_stack_init(cBle_ctrl_on_ble_evt_handler_func evt_cb)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    uint32_t ram_start_app = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    ram_start_app = ram_start;
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    cPrintLog(CDBG_MAIN_LOG, "softdevice enabled! ID:0x%04x, Ram:0x%p,0x%p\n", SD_FWID_GET(MBR_SIZE), (void *)ram_start_app,(void *)ram_start);
    (void)ram_start_app;

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    
    m_ble_evt_cb = evt_cb;
}

//ble advertising
void cfg_ble_advertising_init(uint32_t interval_625_MS_units, uint32_t timeout_sec, cBle_ctrl_on_adv_evt_handler_func adv_evt_cb)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = interval_625_MS_units;
    init.config.ble_adv_fast_timeout  = timeout_sec;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
    m_ble_on_adv_evt_cb = adv_evt_cb;
}

void cfg_ble_advertising_start_stop_req(bool start_flag)
{
    uint32_t err_code;

    if(start_flag)
    {
        err_code = cfg_ble_advertising_start();
        if(err_code == NRF_ERROR_CONN_COUNT)
        {
            cPrintLog(CDBG_BLE_ERR, "advertising_start err! CONN_COUNT\n");
        }
        else if(err_code == NRF_ERROR_INVALID_STATE)
        {
            cPrintLog(CDBG_BLE_ERR, "advertising_start err! INVALID_STATE\n");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    else
    {
        err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
        if(err_code == NRF_ERROR_INVALID_STATE)
        {
            cPrintLog(CDBG_BLE_ERR, "adv_stop err! INVALID_STATE\n");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
            if(m_ble_on_adv_evt_cb)m_ble_on_adv_evt_cb(cBle_ADV_STOP);
        }
    }
    m_ble_advertising_req_state = start_flag;
}

uint32_t cfg_ble_advertising_start(void)
{
    return ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
}

//ble gap_params
void cfg_ble_gap_params_init(const uint8_t *p_dev_name, uint32_t dev_name_size)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)p_dev_name,
                                          dev_name_size);
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        cPrintLog(CDBG_BLE_INFO, "Data len is set to 0x%X(%d)\n", m_ble_max_data_len, m_ble_max_data_len);
    }
    cPrintLog(CDBG_BLE_DBG, "ATT MTU exchange completed. central 0x%x peripheral 0x%x\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

void cfg_ble_gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);  //min BLE_GATT_ATT_MTU_DEFAULT(23), max NRF_SDH_BLE_GATT_MAX_MTU_SIZE
    APP_ERROR_CHECK(err_code);
}

//ble dfu
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            cPrintLog(CDBG_BLE_INFO, "Device is preparing to enter bootloader mode.\n");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler            
            cPrintLog(CDBG_BLE_INFO, "Device will enter bootloader mode.\n");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            cPrintLog(CDBG_BLE_ERR, "Request to enter bootloader mode failed asynchroneously.\n");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            cPrintLog(CDBG_BLE_ERR, "Request to send a response to client failed.\n");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            cPrintLog(CDBG_BLE_ERR, "Unknown event from ble_dfu_buttonless.\n");
            break;
    }
}

//ble services
void cfg_ble_services_init(bool dfu_enable, bool nus_enable, cBle_ctrl_nus_recv_data_handler_func nus_data_cb)
{
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };
    ble_nus_init_t nus_init;

    if(dfu_enable)
    {
        if(m_cfg_available_bootloader_detected)
        {
            cPrintLog(CDBG_BLE_INFO, "dfu enabled!\n");
            // Initialize the async SVCI interface to bootloader.
            err_code = ble_dfu_buttonless_async_svci_init();
            APP_ERROR_CHECK(err_code);

            err_code = ble_dfu_buttonless_init(&dfus_init);
            APP_ERROR_CHECK(err_code);

            /* YOUR_JOB: Add code to initialize the services used by the application.
               uint32_t                           err_code;
               ble_xxs_init_t                     xxs_init;
               ble_yys_init_t                     yys_init;
            
               // Initialize XXX Service.
               memset(&xxs_init, 0, sizeof(xxs_init));
            
               xxs_init.evt_handler                = NULL;
               xxs_init.is_xxx_notify_supported    = true;
               xxs_init.ble_xx_initial_value.level = 100;
            
               err_code = ble_bas_init(&m_xxs, &xxs_init);
               APP_ERROR_CHECK(err_code);
            
               // Initialize YYY Service.
               memset(&yys_init, 0, sizeof(yys_init));
               yys_init.evt_handler                  = on_yys_evt;
               yys_init.ble_yy_initial_value.counter = 0;
            
               err_code = ble_yy_service_init(&yys_init, &yy_init);
               APP_ERROR_CHECK(err_code);
             */
        }
        else
        {
            cPrintLog(CDBG_BLE_ERR, "=== warning! bootloader not detected! dfu disabled! ===\n");
        }
    }

    if(nus_enable)
    {
        memset(&nus_init, 0, sizeof(nus_init));
        nus_init.data_handler = nus_data_handler;
        err_code = ble_nus_init(&m_nus, &nus_init);
        APP_ERROR_CHECK(err_code);
        m_ble_on_nus_data_cb = nus_data_cb;
    }
}

//ble conn_params
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void cfg_ble_conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/******************************************************************************/
// peer manager
/******************************************************************************/
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            cPrintLog(CDBG_BLE_INFO, "Connected to a previously bonded device.\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            cPrintLog(CDBG_BLE_INFO, "Link secured. Role: %d. conn_handle: %d, Procedure: %d\n",
                                 ble_conn_state_role(p_evt->conn_handle),
                                 p_evt->conn_handle,
                                 p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
#if NRF_MODULE_ENABLED(FDS)
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
#endif
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            m_ble_bonds_success_flag = true;
        } break;
        
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

void cfg_ble_peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;
    int timeout;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
    if(erase_bonds)
    {
        nrf_delay_ms(1);
        cPrintLog(CDBG_BLE_INFO, "erase_bonds!\n");
        m_ble_bonds_success_flag = false;
        timeout = 200;
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
        do
        {
            nrf_delay_ms(1);
            if(m_ble_bonds_success_flag)
            {
                break;
            }
        }while(--timeout);
        if(timeout == 0)
        {
            cPrintLog(CDBG_BLE_ERR, "erase_bonds fail!\n");
        }
    }
}

//ble nus
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        cPrintLog(CDBG_BLE_DBG, "Received data from BLE NUS. size:%d\n", p_evt->params.rx_data.length);
        cDataDumpPrintOut(CDBG_BLE_DBG, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        if(m_ble_on_nus_data_cb)m_ble_on_nus_data_cb(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    }
}

uint32_t  cfg_ble_nus_data_send(uint8_t * p_data, uint16_t length)
{
    uint32_t       err_code;
    err_code = ble_nus_data_send(&m_nus, p_data, &length, m_conn_handle);
    return err_code;
}

/******************************************************************************/
// ble scan
/******************************************************************************/
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */
static ble_gap_scan_params_t m_ble_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = true,
};

static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
}

void cfg_ble_scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param = &m_ble_scan_param;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_ble_scan_start(cBle_ctrl_scan_recv_handler_func recv_callback)
{
    ret_code_t ret;

    m_ble_scan_cb = recv_callback;
    m_ble_scan_run_flag = true;
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
}

void cfg_ble_scan_stop(void)
{
    m_ble_scan_cb = NULL;
    m_ble_scan_run_flag = false;

    // Stop scanning.
    nrf_ble_scan_stop();
}


