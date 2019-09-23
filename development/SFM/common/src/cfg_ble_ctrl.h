/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __CFG_BLE_CTRL_H__
#define __CFG_BLE_CTRL_H__
#include <stdbool.h>
#include <stdint.h>
#include "ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool ble_connect_on;
extern bool m_ble_scan_run_flag;
extern bool m_ble_advertising_req_state;
extern bool m_ble_advertising_state;

typedef enum
{
    cBle_GAP_CONNECTED,
    cBle_GAP_DISCONNECTED,
    cBle_event_e_MAX
}cBle_event_e;

typedef enum
{
    cBle_ADV_START,
    cBle_ADV_STOP,
    cBle_adv_event_e_MAX
}cBle_adv_event_e;

typedef void (*cBle_ctrl_on_ble_evt_handler_func)(cBle_event_e cBle_evt, void *p_param);
typedef void (*cBle_ctrl_nus_recv_data_handler_func)(const uint8_t * p_data, uint16_t length);
typedef void (*cBle_ctrl_scan_recv_handler_func)(const ble_gap_evt_adv_report_t * p_adv_report);
typedef void (*cBle_ctrl_on_adv_evt_handler_func)(cBle_adv_event_e cBle_adv_evt);

void cfg_ble_get_ble_mac_address(uint8_t *p_buf /*6byte*/);
void cfg_ble_stack_init(cBle_ctrl_on_ble_evt_handler_func evt_cb);
void cfg_ble_advertising_init(uint32_t interval_625_MS_units, uint32_t timeout_sec, cBle_ctrl_on_adv_evt_handler_func adv_evt_cb);
void cfg_ble_advertising_start_stop_req(bool start_flag);
uint32_t cfg_ble_advertising_start(void);
void cfg_ble_conn_params_init(void);
void cfg_ble_gap_params_init(const uint8_t *p_dev_name, uint32_t dev_name_size);
void cfg_ble_gatt_init(void);
void cfg_ble_services_init(bool dfu_enable, bool nus_enable, cBle_ctrl_nus_recv_data_handler_func nus_data_cb);
void cfg_ble_peer_manager_init(bool erase_bonds);
uint32_t  cfg_ble_nus_data_send(uint8_t * p_data, uint16_t length);

void cfg_ble_scan_init(void);
void cfg_ble_scan_start(cBle_ctrl_scan_recv_handler_func recv_callback);
void cfg_ble_scan_stop(void);

#ifdef __cplusplus
}
#endif
#endif // __CFG_NVM_CTRL_H__
