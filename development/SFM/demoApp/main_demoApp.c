/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_bas.h"
#include "nrf_sdm.h"

#include "cfg_scenario.h"
#include "nordic_common.h"
#include "peer_manager.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "main_demoApp.h"
#include "cfg_config.h"
#include "cfg_sigfox_module.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_gps_module.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "nrf_drv_gpiote.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "hardfault.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_drv_clock.h"
#include "cfg_external_sense_gpio.h"
#include "nrf_ble_gatt.h"
#include "nrf_mbr.h"
#include "cfg_opt3001_module.h"
#include "cfg_encode_tx_data.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_ble_ctrl.h"
#include "cfg_special_boot_mode.h"
#include "cfg_user_cmd_proc.h"
#include "cfg_adc_battery_check.h"
#include "cfg_nRF52_peripherals.h"
#include "cfg_nus_cmd_proc.h"

extern int16_t tmp102a, tmp102b;
extern uint16_t tmp108_data;
extern uint32_t als_lux;

#define USE_UICR_FOR_MAJ_MIN_VALUES 1

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                        /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif
typedef enum
{
    NO_MOTION_STATE,
    MOTION_STATE
}motion_state_t;
APP_TIMER_DEF(m_main_timer_id);

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
bool mSTOPmessage = 0;
bool mWifimsg_send = 0;
#endif

extern uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal

module_mode_t m_module_mode_old = NONE;
module_mode_t m_sigfox_next_state = NONE;
bool m_module_state_started = false;

bool m_init_excute;
int m_module_ready_wait_timeout_tick = 0;

volatile bool main_gps_emer_msg_flag = false;
int main_GPS_BKUP_time_out = 0;  //GPS_BKUP_CTRL

extern uint8_t  magnet_status;
volatile uint32_t motion_sec_tic = 0;
volatile uint32_t no_motion_sec_tic = 0;
volatile uint32_t wait_time_sec_tic = 0;
motion_state_t motion_state = NO_MOTION_STATE;
volatile bool m_wait_retry_event = false;
volatile bool m_wait_retry_sending = false;
volatile uint32_t test_time_sec = 0;
module_peripheral_data_t m_module_tracking_data;

extern bool m_acc_report_to_nus;
extern struct bma_accel_data m_accel;
volatile bool als_init = false;
volatile bool ext_sensor_als_init = false;
volatile bool ext_sensor_tmp_init = false;
bool motion_event = false;
uint32_t downlink_count = 1;
uint32_t downlink_max = 1;

/** @snippet [NFC Launch App usage_0] */
void main_set_module_state(module_mode_t state);
bool main_schedule_state_is_idle(void);
void main_timer_schedule_stop(void);
void module_parameter_check_update(void);
void nus_send_data(char module);

bool get_no_motion_state()
{
    if(motion_state == NO_MOTION_STATE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void set_motion_state()
{
    if(10 < motion_sec_tic)
    {
        if(motion_state != MOTION_STATE)
        {
            cPrintLog(CDBG_FCTRL_INFO, "now in MOTION_STATE state. %d\n",motion_sec_tic);
        }
        motion_state = MOTION_STATE;
        if(m_module_parameter.optional_mode)
            motion_event = true;
    }
        no_motion_sec_tic = 0 ;
}

void set_no_motion_state()
{
    if(m_module_parameter.no_motion_duration < no_motion_sec_tic)
    {
        if(motion_state != NO_MOTION_STATE)
        {
            cPrintLog(CDBG_FCTRL_INFO, "now in NO_MOTION_STATE state. %d\n",no_motion_sec_tic);
        }
        motion_state = NO_MOTION_STATE;
        if(m_module_parameter.optional_mode && motion_event)
        {
           motion_event = false;
           cfg_scen_wakeup_request(main_wakeup_reason_acc_event_motion);
        }
//      cPrintLog(CDBG_FCTRL_INFO, "NO MOTION STATE\n");
    }
    motion_sec_tic = 0;
}

void set_wait_time_state()
{
    wait_time_sec_tic = 0;
}

bool get_wait_time_state()
{
    if(m_module_parameter.wait_time_state < wait_time_sec_tic)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void on_cBle_evt(cBle_event_e cBle_evt, void *p_param)
{
    switch (cBle_evt)
    {
        case cBle_GAP_CONNECTED:
            break;
        case cBle_GAP_DISCONNECTED:
            if(m_acc_report_to_nus)
            {
                m_acc_report_to_nus = false;
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}

static bool module_parameter_ID_value_update(void)
{
    bool ret = false;
    if((memcmp(m_module_peripheral_ID.wifi_MAC_STA, m_module_parameter.wifi_MAC_STA, sizeof(m_module_peripheral_ID.wifi_MAC_STA)) != 0)
        || (memcmp(m_module_peripheral_ID.sigfox_device_ID, m_module_parameter.sigfox_device_ID, sizeof(m_module_peripheral_ID.sigfox_device_ID)) != 0)
        || (memcmp(m_module_peripheral_ID.sigfox_pac_code, m_module_parameter.sigfox_pac_code, sizeof(m_module_peripheral_ID.sigfox_pac_code)) != 0)
    )
    {
        cPrintLog(CDBG_FLASH_INFO, "update ID Value!\n");
        //for ID value cache
        memcpy(m_module_parameter.wifi_MAC_STA, m_module_peripheral_ID.wifi_MAC_STA, sizeof(m_module_parameter.wifi_MAC_STA));
        memcpy(m_module_parameter.sigfox_device_ID, m_module_peripheral_ID.sigfox_device_ID, sizeof(m_module_parameter.sigfox_device_ID));
        memcpy(m_module_parameter.sigfox_pac_code, m_module_peripheral_ID.sigfox_pac_code, sizeof(m_module_parameter.sigfox_pac_code));

        module_parameter_update();
        ret = true;
    }
    return ret;
}

void module_make_cached_tracking_data(void)
{
    uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};
    cPrintLog(CDBG_FCTRL_INFO, "cached tracking data\n");
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
    if(m_module_tracking_data.wifi_status)
    {
        if(m_module_tracking_data.wifi_match_gps_status)
        {
            encode_12byte_dataV2_PosData(main_send_data_wifi_pos, send_data, main_wakeup_reason, m_module_tracking_data.wifi_match_gps_data, batt_avg_report_percent, magnet_status);
        }
        else
        {
            encode_12byte_dataV2_WIFI(send_data, main_wakeup_reason, m_module_tracking_data.wifi_data, m_module_tracking_data.wifi_rssi[0], batt_avg_report_percent, magnet_status);
        }
    }
    else if(m_module_tracking_data.ble_scan_status)
    {
        if(m_module_tracking_data.ble_scan_match_gps_status)
        {
            encode_12byte_dataV2_PosData(main_send_data_ble_pos, send_data, main_wakeup_reason, m_module_tracking_data.ble_scan_match_gps_data, batt_avg_report_percent, magnet_status);
        }
        else
        {
            encode_12byte_dataV2_BLE(send_data, main_wakeup_reason, m_module_tracking_data.ble_scan_data, m_module_tracking_data.ble_scan_rssi[0], batt_avg_report_percent, magnet_status);
        }
    }
    else if(m_module_tracking_data.gps_status)
    {
        encode_12byte_dataV2_GPS(send_data, main_wakeup_reason, m_module_tracking_data.gps_data, batt_avg_report_percent, magnet_status);
    }
    else
    {
        encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, magnet_status);
    }
#else
    if(m_module_tracking_data.gps_status)
    {
        encode_12byte_dataV2_GPS(send_data, main_wakeup_reason, m_module_tracking_data.gps_data, batt_avg_report_percent, tmp108_data);
    }
    else if(m_module_tracking_data.ble_scan_status)
    {
        if(m_module_tracking_data.ble_scan_match_gps_status)
        {
            encode_12byte_dataV2_PosData(main_send_data_ble_pos, send_data, main_wakeup_reason, m_module_tracking_data.ble_scan_match_gps_data, batt_avg_report_percent, tmp108_data);
        }
        else
        {
            encode_12byte_dataV2_BLE(send_data, main_wakeup_reason, m_module_tracking_data.ble_scan_data, m_module_tracking_data.ble_scan_rssi[0], batt_avg_report_percent, tmp108_data);
        }
    }
    else if(m_module_tracking_data.wifi_status)
    {
        if(m_module_tracking_data.wifi_match_gps_status)
        {
            encode_12byte_dataV2_PosData(main_send_data_wifi_pos, send_data, main_wakeup_reason, m_module_tracking_data.wifi_match_gps_data, batt_avg_report_percent, tmp108_data);
        }
        else
        {
            encode_12byte_dataV2_WIFI(send_data, main_wakeup_reason, m_module_tracking_data.wifi_data, m_module_tracking_data.wifi_rssi[0], batt_avg_report_percent, tmp108_data);
        }
    }
    else
    {
        encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, tmp108_data);
    }
#endif
    cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
    cPrintLog(CDBG_FCTRL_INFO, "replace sigfox payload for cached_tracking_data:[%s]\n",frame_data);
}

void module_make_sensor_data(void)
{
    uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};

    cPrintLog(CDBG_FCTRL_INFO, "sensor data\n");
    encode_12byte_dataV2_Sensor(send_data, main_wakeup_reason, &m_accel, als_lux, batt_avg_report_percent, tmp108_data);
    cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
    cPrintLog(CDBG_FCTRL_INFO, "replace sigfox payload for sensor data:[%s]\n",frame_data);
}

static void on_cBle_adv_evt(cBle_adv_event_e cBle_adv_evt)
{
    switch (cBle_adv_evt)
    {
        case cBle_ADV_START:
            cPrintLog(CDBG_FCTRL_INFO, "Adv Started\n");
            break;

        case cBle_ADV_STOP:
            cPrintLog(CDBG_FCTRL_INFO, "Adv Stopped\n");
            break;
    
        default:
            break;
    }
}

static void advertising_info_init(void)
{
    char device_name[32];
    uint32_t dev_name_size;
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE)
    sprintf(device_name, "%s_V%s", "iHere", m_cfg_sw_ver);
#else
    //default advertising
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)
    sprintf(device_name, "SFM60R");
#elif (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)
    sprintf(device_name, "SRM200A");
#else
    sprintf(device_name, "SFM20R");  //default advertising
#endif
#endif
    dev_name_size = strlen(device_name);
    cfg_ble_gap_params_init((const uint8_t *)device_name, dev_name_size);
}

/**
 * @brief Function for starting idle timer.
 */
void main_timer_idle_start(void)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(1000*m_module_parameter.idle_time), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for starting main schedule timer.
 *
*/
void main_timer_schedule_start(void)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for restarting main schedule at idle state
 *
*/
void main_timer_schedule_restart_check_idle(void)
{
    if(main_schedule_state_is_idle())
    {
        main_timer_schedule_stop();
        main_timer_schedule_start();
    }
}

/**
 * @brief Function for starting variable timer.
 */
void main_timer_variable_schedule_start(unsigned int schedule_time_sec)
{
    uint32_t      err_code;

    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(1000*schedule_time_sec), NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for stop  main schedule timer.
 */
void main_timer_schedule_stop(void)
{
    uint32_t      err_code;
    err_code = app_timer_stop(m_main_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting main state in main scheduler.
 *
 * @param[in] state  set the next state in main scheduler.
 *                         
 */

void main_set_module_state(module_mode_t state)
{
    m_init_excute = true; /*for run initial*/
    m_module_ready_wait_timeout_tick = 0; /*for timeout check*/
    cPrintLog(CDBG_FCTRL_DBG, "main_set_module_state %d to %d\n", m_module_mode, state);
    m_module_mode = state;
}

void main_set_module_state_goto_do_nothing(void)
{
    m_module_mode = MAX_DO_NOTHING;
}

module_mode_t main_get_module_state_next(bool normal)
{
    module_mode_t ret;
    switch(m_module_mode)
    {
        case NONE:  //first state
            ret = ACC;
            break;
        case ACC:
            ret = MAIN_SCENARIO_LOOP;
            break;
        case MAIN_SCENARIO_LOOP:  //event start state
            ret = BATTERY_CHECK;
            break;
        case BATTERY_CHECK:
            ret = TMP;
            break;
        case TMP:
            ret = ALS;
            break;
        case ALS:
            ret = GPS;
            break;
        case BLE:
            ret = IDLE;
            break;
        case GPS:
            if(normal)  //gps data scaned
            {
                if(m_module_parameter.operation_mode == 2/*full tracking mode*/ || test_nus_full_tracking_mode)  //It is for engineer mode.
                {
                    ret = BLE_SCAN;
                }
                else
                {
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                    ret = SIGFOX_CHECK_RUN_SCAN_RC;
#else
                    ret = SIGFOX;
#endif

                }
            }
            else
            {
                ret = BLE_SCAN;
            }
            break;
        case BLE_SCAN:
            if(normal)  //ble beacon data scaned
            {
                if(m_module_parameter.operation_mode == 2/*full tracking mode*/ || test_nus_full_tracking_mode)  //It is for engineer mode.
                {
                    ret = WIFI;
                }
                else
                {
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                    ret = SIGFOX_CHECK_RUN_SCAN_RC;
#else
                    ret = SIGFOX;
#endif
                }
            }
            else
            {
                ret = WIFI;
            }
            break;
        case WIFI:
            if(normal)  //wifi AP data scaned
            {
                if(m_module_parameter.operation_mode == 2/*full tracking mode*/ || test_nus_full_tracking_mode)  //It is for engineer mode.
                {
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                    ret = SIGFOX_CHECK_RUN_SCAN_RC;
#else
                    ret = SIGFOX;
#endif
                }
                else
                {
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                    ret = SIGFOX_CHECK_RUN_SCAN_RC;
#else
                    ret = SIGFOX;
#endif
                }
            }
            else
            {
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                ret = SIGFOX_CHECK_RUN_SCAN_RC;
#else
                ret = SIGFOX;
#endif
            }
            break;
        case SIGFOX:
            ret = BLE;
            break;
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
        case SIGFOX_CHECK_RUN_SCAN_RC:
            if(normal)
            {
                ret = SIGFOX_SCAN_RC;
            }
            else
            {
                ret = SIGFOX;  //not send sigfox data
            }
            break;

        case SIGFOX_SCAN_RC:
            if(normal)
            {
                ret = SIGFOX;
            }
            else
            {
                ret = BLE;  //not send sigfox data
            }
            break;
#endif
        case IDLE:
            ret = MAIN_SCENARIO_LOOP;
            break;
        default:
            cPrintLog(CDBG_FCTRL_ERR, "Known next state\n");
            APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
            ret = MAIN_SCENARIO_LOOP;
            break;
    }
    return ret;
}

/**@brief Function for indicating whether the current state is idle or not.
 *
 * @return true if it is idle, nor false
 *                         
 */

bool main_schedule_state_is_idle(void)
{
    if(m_module_mode == IDLE)
        return true;
    else
        return false;
}
bool main_schedule_state_is_sigfox(void)
{
    if(m_module_mode == SIGFOX)
        return true;
    else
        return false;
}

#ifdef USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
void main_magnet_attach_callback(void)
{
    main_magnet_detected = true;
    cTBC_event_noti("MAGNET_CLOSE");

    magnet_status = 1;
    if((m_module_parameter.magnet_operation_mode & 0x01))
    {
        cfg_scen_wakeup_request(main_wakeup_reason_magnetic_event_att);
    }
}

void main_magnet_detach_callback(void)
{
    magnet_status = 0;
    cTBC_event_noti("MAGNET_OPEN");
    if((m_module_parameter.magnet_operation_mode & 0x02))
    {
        cfg_scen_wakeup_request(main_wakeup_reason_magnetic_event_det);
    }
}
#endif

#ifdef USR_MODULE_GPIO_DEF_WAKEUP_KEY
void main_wkup_det_callback(void)
{
    main_wkup_key_detected = true;
    cTBC_event_noti("WKUP_KEY");
    cfg_scen_wakeup_request(main_wakeup_reason_key_event);
}
#endif

#ifdef USR_MODULE_GPIO_DEF_BUTTON
void main_button_short_press_callback(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "button short event\n");
    main_button_detected = true;
    cfg_scen_wakeup_request(main_wakeup_reason_key_event);
}

void main_button_long_press_callback(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "button long event\n");
#ifdef USR_MODULE_GPIO_DEF_BUTTON_POWER_OFF
    cfg_scen_powerdown_request(0, true);
#endif
}
#endif

static void cache_gps_data(const uint8_t * pos_data)  //print pos_data and LatitudeLongitude
{   
    char latitude_str_buf[11], longitude_str_buf[12];
    if(make_PosData_to_LatitudeLongitude(pos_data, latitude_str_buf, longitude_str_buf))
    {
        cPrintLog(CDBG_GPS_INFO, "pos data:%02x%02x%02x%02x%02x%02x%02x%02x\n",pos_data[0],pos_data[1],pos_data[2],pos_data[3],pos_data[4],pos_data[5],pos_data[6],pos_data[7]);
        cPrintLog(CDBG_GPS_INFO, "LatiLongi:[%s, %s]\n", latitude_str_buf, longitude_str_buf);
        m_module_tracking_data.gps_status = 1;
        memcpy(m_module_tracking_data.gps_data, pos_data, sizeof(m_module_tracking_data.gps_data));
#if 0  //recheck pos data
        {
            uint8_t pos_check_buf[8];
            memset(pos_check_buf, 0, sizeof(pos_check_buf));
            make_LatitudeLongitude_to_PosData(latitude_str_buf, longitude_str_buf, pos_check_buf);
            cPrintLog(CDBG_FCTRL_INFO, "pos data check:");
            cDataDumpPrintOut(CDBG_FCTRL_INFO, pos_check_buf, 8);
        }
#endif
    }
    else
    {
        cPrintLog(CDBG_FCTRL_INFO, "pos data error\n");
    }
}

#ifdef CDEV_BLE_SCAN_ENABLE
#define BLE_SCAN_Developer_logs (0)

#define BLE_SCAN_TIME_TICK (5*5) //about 5 Sec
#define BLE_SCAN_CNT_MAX 4

typedef struct
{
    uint8_t item_cnt;
    int8_t lowest_rssi;
    ble_gap_evt_adv_report_t adv_info[BLE_SCAN_CNT_MAX];
}m_blescan_data_info_t;
m_blescan_data_info_t m_blescan_info;
m_blescan_data_info_t m_blescan_info_pri; //for mac list

#if (BLE_SCAN_Developer_logs)
m_blescan_data_info_t m_blescan_info_old;
m_blescan_data_info_t m_blescan_info_pri_old;
#endif

bool main_ble_scan_info_check_exists(m_blescan_data_info_t *p_info_list, const ble_gap_evt_adv_report_t *p_cur_item, uint32_t *get_idx)
{

    bool ret = false;
    int i;
    uint32_t idx = 0;
    for(i = 0; i < p_info_list->item_cnt; i++)
    {
        if(memcmp(p_info_list->adv_info[i].peer_addr.addr, p_cur_item->peer_addr.addr, BLE_GAP_ADDR_LEN) == 0)
        {
            ret = true;
            idx = (uint32_t)i;
            break;
        }
    }
    if(get_idx)*get_idx = idx;
    return ret;
}

void main_ble_scan_info_update_rssi(uint32_t idx, int8_t cur_rssi, m_blescan_data_info_t *p_info_list)
{
    int8_t list_rssi;
    int  i;
    uint32_t item_cnt;

    item_cnt = p_info_list->item_cnt;
    list_rssi = p_info_list->adv_info[idx].rssi;

    //update rssi for higher value
    if(list_rssi < cur_rssi)
    {
        p_info_list->adv_info[idx].rssi = cur_rssi;
        //update lowset rssi
        if(list_rssi == p_info_list->lowest_rssi)
        {
            p_info_list->lowest_rssi = 0;            
            for(i=0; i < item_cnt; i++)
            {
                if(p_info_list->lowest_rssi > p_info_list->adv_info[i].rssi)
                {
                    p_info_list->lowest_rssi = p_info_list->adv_info[i].rssi;
                }
            }
        }
    }

}

static void main_ble_scan_info_update(m_blescan_data_info_t *p_info_list, const ble_gap_evt_adv_report_t *p_cur_item)
{
    bool exists = false;
    uint32_t item_cnt, idx;
    int i;
    
    item_cnt = p_info_list->item_cnt;
    if(item_cnt)
    {
        exists = main_ble_scan_info_check_exists(p_info_list, p_cur_item, &idx);
    }

    if(exists)
    {
        main_ble_scan_info_update_rssi(idx, p_cur_item->rssi, p_info_list);
    }
    else
    {
        if(item_cnt < BLE_SCAN_CNT_MAX)
        {
            //add belscan data
            memcpy(&(p_info_list->adv_info[item_cnt]), p_cur_item, sizeof(ble_gap_evt_adv_report_t));
            p_info_list->item_cnt++;
            if(p_cur_item->rssi < p_info_list->lowest_rssi)
            {
                p_info_list->lowest_rssi = p_cur_item->rssi;
            }
        }
        else
        {
            //replace blescan data
            if(p_cur_item->rssi > p_info_list->lowest_rssi)
            {
                int8_t rssi_cmp;
                uint32_t lowest_rssi_item_idx;
                
                //search lowest rssi item
                rssi_cmp = p_info_list->adv_info[0].rssi;
                lowest_rssi_item_idx = 0;
                for(i=0; i < item_cnt; i++)
                {
                    if(rssi_cmp > p_info_list->adv_info[i].rssi)
                    {
                        lowest_rssi_item_idx = i;
                        p_info_list->lowest_rssi = p_info_list->adv_info[i].rssi;
                    }
                }

                //replace lowest rssi item
                memcpy(&p_info_list->adv_info[lowest_rssi_item_idx], p_cur_item, sizeof(ble_gap_evt_adv_report_t));

                p_info_list->lowest_rssi = 0;            
                for(i=0; i < item_cnt; i++)
                {
                    if(p_info_list->lowest_rssi > p_info_list->adv_info[i].rssi)
                    {
                        p_info_list->lowest_rssi = p_info_list->adv_info[i].rssi;
                    }
                }
            }
        }
    }
}

static void main_ble_scan_info_init(void)
{
    memset(&m_blescan_info, 0, sizeof(m_blescan_info));
    memset(&m_blescan_info_pri, 0, sizeof(m_blescan_info_pri));
#if (BLE_SCAN_Developer_logs)
    memset(&m_blescan_info_old, 0, sizeof(m_blescan_info_old));
    memset(&m_blescan_info_pri_old, 0, sizeof(m_blescan_info_pri_old));
#endif
}

static void main_ble_scan_handler(const ble_gap_evt_adv_report_t * p_adv_report)
{
    uint32_t idx;

#if (BLE_SCAN_Developer_logs)
    {
        const uint8_t *addr = p_adv_report->peer_addr.addr;
        cPrintLog(CDBG_BLE_INFO,"Scaned! Rssi:%d, Mac:%02x%02x%02x%02x%02x%02x\n", p_adv_report->rssi, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        (void)addr;
    }
#endif
    //update m_blescan_info
    main_ble_scan_info_update(&m_blescan_info, p_adv_report);

    //update m_blescan_info_pri (for mac list)
    if(main_ble_scan_info_check_exists(&m_blescan_info_pri, p_adv_report, &idx))
    {
        main_ble_scan_info_update_rssi(idx, p_adv_report->rssi, &m_blescan_info_pri);
    }
    else
    {
        int i;
        const uint8_t *p_addr;
        uint8_t cur_mac_buf[6];

        p_addr = p_adv_report->peer_addr.addr;
        //reverse mac addr
        cur_mac_buf[0] = p_addr[5];
        cur_mac_buf[1] = p_addr[4];
        cur_mac_buf[2] = p_addr[3];
        cur_mac_buf[3] = p_addr[2];
        cur_mac_buf[4] = p_addr[1];
        cur_mac_buf[5] = p_addr[0];
                                                
        for(i = 0; i < m_registered_mac_info.ble_mac_cnt; i++)
        {
            if(memcmp(m_registered_mac_info.ble_r_mac[i], cur_mac_buf, 6) == 0)
            {
                main_ble_scan_info_update(&m_blescan_info_pri, p_adv_report);
                break;
            }
        }
    }
#if (BLE_SCAN_Developer_logs)
    {
        int i;
        uint8_t *addr;

        if(memcmp(&m_blescan_info, &m_blescan_info_old, sizeof(m_blescan_info)))
        {
            cPrintLog(CDBG_BLE_INFO,"m_blescan_info updated! [%d], [%d]\n", (uint8_t)m_blescan_info.item_cnt, (int8_t)m_blescan_info.lowest_rssi);
            for(i = 0; i < m_blescan_info.item_cnt; i++)
            {
                addr = m_blescan_info.adv_info[i].peer_addr.addr;
                cPrintLog(CDBG_BLE_INFO,"  [%d] [%d] [%02x%02x%02x%02x%02x%02x]\n", i, (int8_t)m_blescan_info.adv_info[i].rssi, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            }
            memcpy(&m_blescan_info_old, &m_blescan_info, sizeof(m_blescan_info_old));
        }
        if(memcmp(&m_blescan_info_pri, &m_blescan_info_pri_old, sizeof(m_blescan_info_pri)))
        {
            cPrintLog(CDBG_BLE_INFO,"m_blescan_info_pri updated!\n");
            for(i = 0; i < m_blescan_info_pri.item_cnt; i++)
            {
                addr = m_blescan_info_pri.adv_info[i].peer_addr.addr;
                cPrintLog(CDBG_BLE_INFO,"  [%d] [%d] [%02x%02x%02x%02x%02x%02x]\n", i, (int8_t)m_blescan_info_pri.adv_info[i].rssi, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            }
            memcpy(&m_blescan_info_pri_old, &m_blescan_info_pri, sizeof(m_blescan_info_pri_old));
        }
        (void)addr;
    }
#endif
}
#endif
/**
 * @brief Callback function for handling main state  from main schedule timer.
 * @details manage main scheduling in normal demo
 */

static void main_schedule_timeout_handler_asset_tracker(void * p_context)
{
    UNUSED_PARAMETER(p_context);
#ifdef CDEV_GPS_MODULE
    bool send_gps_data_req = false;
    uint8_t gps_send_data[SIGFOX_SEND_PAYLOAD_SIZE];
    int get_nmea_result = 0;
    uint8_t *nmea_Buf;
    static int work_mode = GPS_START;  //module_work_e
#endif
    static bool old_ble_led = false;

    if(m_module_mode != m_module_mode_old)
    {
        cPrintLog(CDBG_FCTRL_DBG, "main state changed %d to %d\n", m_module_mode_old, m_module_mode);
        m_module_mode_old = m_module_mode;
    }

    switch(m_module_mode)
    {
        case ACC:
            if(cfg_peripheral_device_enable_status_get(PTYPE_ACC))
            {
#ifndef CDEV_DISABLE_ACC_MODULE_LIB
                if(bma250_get_state() == NONE_ACC)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "ACC Start\n");
                    if(m_module_parameter.interrupt_src == 4)//(m_module_parameter.operation_mode == 0&&m_module_parameter.acc_operation_mode == 1)
                    {
                        m_acc_report_to_nus = true; 
                        cPrintLog(CDBG_FCTRL_INFO, "ACC Nus Flag Enabled!\n");
                    }
                    bma250_set_state(SET_S);
                    cfg_bma250_timers_start();
                }
                else if(bma250_get_state() == EXIT_ACC)
                {
                    cfg_bma250_timers_stop();
    //                bma250_set_state(NONE_ACC);
                }
#endif
                main_set_module_state(main_get_module_state_next(true));
            }
            else
            {
                main_set_module_state(main_get_module_state_next(false));
            }
            break;

        case MAIN_SCENARIO_LOOP:
            cPrintLog(CDBG_FCTRL_INFO, "==MAIN_SCENARIO_LOOP %u==\n", main_Sec_tick);
            cfg_ble_led_control(true);
            main_set_module_state(main_get_module_state_next(true));
            cfg_board_common_power_control(module_comm_pwr_common, true);
            memset(&m_module_tracking_data, 0, sizeof(m_module_tracking_data));
            break;

        case BATTERY_CHECK:
#ifdef USR_MODULE_DEF_BATTERY_ADC_INPUT
            ++m_module_ready_wait_timeout_tick;

            if(m_init_excute)
            {
                cfg_battery_check_start();
                m_init_excute = false;
            }
            else
            {
                if(m_module_ready_wait_timeout_tick > (APP_MAIN_SCHEDULE_HZ/2))  //wait 500ms
                {
                    bool log_once_flag = false;
                    uint16_t avg_batt_lvl_in_milli_volts;

                    if(m_module_ready_wait_timeout_tick == (APP_MAIN_SCHEDULE_HZ * 2))log_once_flag=true;
#ifdef FEATURE_CFG_BLE_UART_CONTROL
                    m_nus_service_parameter.battery_volt[0] = batt_avg_report_volts;
                    m_nus_service_parameter.module='B';
#endif
                    avg_batt_lvl_in_milli_volts = get_avg_batt_lvl_in_milli_volts();
                    if(BATTERY_ADC_INPUT_MIN <= avg_batt_lvl_in_milli_volts && 5200 >= avg_batt_lvl_in_milli_volts)
                    {
                        if(m_module_parameter.disable_battery_power_down)cPrintLog(CDBG_MAIN_LOG, "Battery Pwr Off Disabled\n");
                        if(!m_module_parameter.disable_battery_power_down && !cTBC_check_host_connected() && avg_batt_lvl_in_milli_volts < BATTERY_ADC_CUT_OFF_LVL)  //low battery
                        {
                            //battery value is enough
                            if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "Battery Low:%d\n", avg_batt_lvl_in_milli_volts);
                            cfg_scen_powerdown_request(0, false);
                            main_set_module_state_goto_do_nothing();  //Do not do anything until the power is turned off.
                        }
                        else if(!m_module_parameter.disable_battery_power_down && avg_batt_lvl_in_milli_volts <= BATTERY_ADC_WARNING_LVL)  //battery warning
                        {
                            if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "Battery Warning:%d\n", avg_batt_lvl_in_milli_volts);
                            if(m_module_ready_wait_timeout_tick > (APP_MAIN_SCHEDULE_HZ * 4))  //wait more 2 sec for warning noti
                            {
                                main_set_module_state(main_get_module_state_next(true));  //battery value is enough
                            }
                            else
                            {
                                if(((m_module_ready_wait_timeout_tick % APP_MAIN_SCHEDULE_HZ) == 0))
                                {
                                    cfg_ble_led_control(false);
                                }
                                else if(((m_module_ready_wait_timeout_tick % APP_MAIN_SCHEDULE_HZ) == 2))
                                {
                                    cfg_ble_led_control(true);
                                }
                            }
                        }
                        else
                        {
                            main_set_module_state(main_get_module_state_next(true));   //battery value is enough
                        }
                    }
                    else
                    {
                        if(log_once_flag)cPrintLog(CDBG_MAIN_LOG, "ADC Not Available:%d\n", avg_batt_lvl_in_milli_volts);
                        main_set_module_state(main_get_module_state_next(true));  //battery value is not available
                    }
                }
                m_nus_service_parameter.battery_volt[0] = batt_avg_report_volts;
                m_nus_service_parameter.module='B';
            }
#else
            main_set_module_state(main_get_module_state_next(false));
#endif
            break;

        case TMP:
            if(0){}  //dummy if for ifdef
#if defined(CDEV_TEMPERATURE_SENSOR_TMP102) || defined(CDEV_TEMPERATURE_SENSOR_TMP108)
            else if(cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE))
            {
                if(!ext_sensor_tmp_init)
                {
                    if(tmp102_get_state() == NONE_TMP)
                    {
                        cTBC_write_state_noti("Temperature");
                        cPrintLog(CDBG_FCTRL_INFO, "Temperature Start 1\n");
                        tmp102_set_state(TMP_SET_S);
                        cfg_tmp102_timers_start();
                    }
                    else if(tmp102_get_state() == EXIT_TMP)
                    {
                        cfg_tmp102_timers_stop();
                        tmp102_set_state(NONE_TMP);
                        main_set_module_state(main_get_module_state_next(true));
                        ext_sensor_tmp_init = true;
                    }
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "Temperature Start 2\n");
                    tmp102_set_state(TMP_READ_ONLY_S);
                    cfg_tmp102_timers_start();
                    main_set_module_state(main_get_module_state_next(true));
                }

            }
#endif
            else
            {
                cPrintLog(CDBG_FCTRL_INFO, "Temperature disabled!\n");
                tmp108_data = 0xffff;
                main_set_module_state(main_get_module_state_next(false));
            }
            break;
        case ALS:
            if(0){}  //dummy if for ifdef
#ifdef CDEV_AMBIENT_LIGHT_SENSOR
            else if(cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
            {
                if(!ext_sensor_als_init)
                {
                    if(opt3001_get_state() == NONE_ALS)
                    {
                        m_init_excute = false;
                        cfg_ble_led_control(false);
                        cPrintLog(CDBG_FCTRL_INFO, "Ambient Light Start 1\n");
                        opt3001_set_state(ALS_SET_S);
                        cfg_opt3001_timers_start();
                    }
                    else if(opt3001_get_state() == EXIT_ALS)
                    {
                        cfg_opt3001_timers_stop();
                        opt3001_set_state(NONE_ALS);
                        cfg_ble_led_control(true);
                        main_set_module_state(main_get_module_state_next(true));
                        ext_sensor_als_init = true;
                    }
                }
                else
                {
                    cfg_ble_led_control(false);
                    cPrintLog(CDBG_FCTRL_INFO, "Ambient Light Start 2\n");
                    opt3001_set_state(ALS_READ_ONLY_S);
                    cfg_opt3001_timers_start();
                    nrf_delay_ms(200); // led off delay time
                    cfg_ble_led_control(true);
                    main_set_module_state(main_get_module_state_next(true));
                }
            }
#endif
            else
            {
            #if 0
                if(m_init_excute)
                {
                    m_init_excute = false;
                    cfg_ble_led_control(false);
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "ALS disabled!\n");
                    cfg_ble_led_control(true);
                    main_set_module_state(main_get_module_state_next(false));
                }
            #endif    
                cPrintLog(CDBG_FCTRL_INFO, "Ambient Light disabled!\n");
                als_lux = 0xffff;    
                //cfg_ble_led_control(true);
                main_set_module_state(main_get_module_state_next(false));
            }
            break;
            
        case BLE:
            cTBC_write_state_noti("BleAdvertising");
            cPrintLog(CDBG_BLE_INFO, "Normal Sleep Start\n");
            cfg_ble_led_control(false);
#ifdef CDEV_GPS_MODULE
            cGps_power_control(false, false);  //gps power off
#endif
#ifdef FEATURE_CFG_BLE_UART_CONTROL
            m_nus_service_parameter.magnet_event = '0';
            m_nus_service_parameter.accellometer_event = '0';
#endif
            if(test_nus_full_tracking_mode)test_nus_full_tracking_mode = false;
            main_wakeup_reason = main_wakeup_reason_normal;  //clear event reason
            main_timer_schedule_stop();
            if(!m_module_parameter.optional_mode)
                main_timer_idle_start();
            main_set_module_state(main_get_module_state_next(true));
            cfg_board_common_power_control(module_comm_pwr_common, false);
            break;
        case GPS:
#ifdef CDEV_GPS_MODULE
            memset(gps_send_data, 0, sizeof(gps_send_data));

            if(cfg_peripheral_device_enable_status_get(PTYPE_GPS))
            {
                if(work_mode == GPS_START)
                {
                    main_GPS_BKUP_time_out = COLD_START_TIME;  //GPS_BKUP_CTRL
                    cfg_led_2_control(true);
                    if(cGps_status_available() == CGPS_Result_OK)
                    {
                        if(!cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
                        {
                            cfg_ble_led_control(false);
                        }
                        cTBC_write_state_noti("GPS");
                        cPrintLog(CDBG_FCTRL_INFO, "GPS Start\n");
                        cGps_nmea_acquire_request();          
                        work_mode = GPS_WORK; //wait scan
                    }
                    else
                    {
                        work_mode = GPS_END;
                    }
                }
                else if(work_mode == GPS_WORK)
                {
                    if(!cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
                    {
                        if(!cfg_get_ble_led_status())
                        {
                            cfg_ble_led_control(true);
                        }
                    }
                    get_nmea_result = cGps_acquire_tracking_check();
                    if(get_nmea_result == CGPS_Result_OK)
                    {
                        work_mode = GPS_END;
                    }
                    else  if(get_nmea_result ==  CGPS_Result_Busy)
                    {
                        ;
                    }
                    else
                    {
                        if(!cGps_bus_busy_check())
                        {
                            work_mode = GPS_START;
                            cfg_led_2_control(false);
                            main_set_module_state(main_get_module_state_next(false));
                            nus_send_data('G');
                        }
                    }
                }
                else if(work_mode == GPS_END)
                {
                    get_nmea_result = cGps_nmea_get_bufPtr(&nmea_Buf);
                    if(get_nmea_result == CGPS_Result_OK)
                    {
                        memcpy(gps_send_data, nmea_Buf, sizeof(gps_send_data));
                        send_gps_data_req = true;
                    }
                    if(send_gps_data_req)
                    {

                        gps_send_data[9] = SIGFOX_MSG_SEND_REASON(main_wakeup_reason);  /*10th bype Status filed modifyed*/
#if 1  //This is a temporary code. must be implement the code.
                        {
//                            if((gps_send_data[0] != 'N') && (gps_send_data[0] != 'S'))
                            {
                                uint8_t gpsdata[8];
                                memcpy(gpsdata, &gps_send_data[0], 8);
                                cache_gps_data(gpsdata);  //print pos_data and LatitudeLongitude
#if  (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                                encode_12byte_dataV2_GPS(gps_send_data, main_wakeup_reason, gpsdata, batt_avg_report_percent, magnet_status);
#else
                                encode_12byte_dataV2_GPS(gps_send_data, main_wakeup_reason, gpsdata, batt_avg_report_percent, tmp108_data);
#endif
                            }
                        }
#endif
                        cfg_bin_2_hexadecimal(gps_send_data, SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);  //set sigfox payload
                        cPrintLog(CDBG_FCTRL_INFO, "[GPS] %d send request sigfox! frame_data:[%s]  \n", __LINE__, frame_data);
                        send_gps_data_req = false;
                        work_mode = GPS_START;
#ifdef FEATURE_CFG_BLE_UART_CONTROL
          //              memcpy(m_nus_service_parameter.gps_data, gps_send_data, sizeof(m_nus_service_parameter.gps_data));
                        m_nus_service_parameter.module='G';
#endif
                        cfg_led_2_control(false);
                        main_set_module_state(main_get_module_state_next(true));
                    }
                    else
                    {
                        if(!cGps_bus_busy_check())
                        {
                            work_mode = GPS_START;
                            cfg_led_2_control(false);
                            main_set_module_state(main_get_module_state_next(false));
                        }
                    }
                    nus_send_data('G');
                }
            }
            else
            {
                cPrintLog(CDBG_FCTRL_INFO, "GPS disabled!\n");
                cfg_led_2_control(false);
                main_set_module_state(main_get_module_state_next(false));
            }
#else
            cPrintLog(CDBG_FCTRL_INFO, "CDEV_GPS_MODULE Not Defined!\n");
            cfg_led_2_control(false);
            main_set_module_state(main_get_module_state_next(false));
#endif
            break;

        case BLE_SCAN:
            {
                bool ble_scan_go_next_state = false;
                bool bScaned = false;
                static bool old_adv_state = false;

#ifdef CDEV_BLE_SCAN_ENABLE
                if(cfg_peripheral_device_enable_status_get(PTYPE_BLE_SCAN))
                {
                    if(m_init_excute)
                    {
                        if(ble_connect_on)
                        {
                            uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};
                            cPrintLog(CDBG_FCTRL_INFO, "ble busy. gap connected!\n");
                            ble_scan_go_next_state = true;
                            //No data
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                            encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, magnet_status);
#else
                            encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, tmp108_data);
#endif
                            cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                        }
                        else
                        {
                            if(m_module_ready_wait_timeout_tick == 0)
                            {
                                if(m_ble_advertising_req_state)
                                {
                                    old_adv_state = true;
                                    cPrintLog(CDBG_FCTRL_DBG, "adv stop for ble scan\n");
                                    cfg_ble_advertising_start_stop_req(false);
                                }
                                else
                                {
                                    old_adv_state = false;
                                }
                            }
                            else
                            {
                                cPrintLog(CDBG_FCTRL_INFO, "BLE_SCAN Start\n");
                                main_ble_scan_info_init();
                                cfg_ble_scan_start(main_ble_scan_handler);
                                m_init_excute = false;
                            }
                        }
                    }
                    else
                    {
                        if(m_module_ready_wait_timeout_tick >= BLE_SCAN_TIME_TICK)
                        {
                            if(m_module_ready_wait_timeout_tick == BLE_SCAN_TIME_TICK)
                            {
                                cfg_ble_scan_stop();
                            }
                            else
                            {
                                cPrintLog(CDBG_FCTRL_INFO, "BLE_SCAN end. cached:[%d],list:[%d]\n", m_blescan_info.item_cnt, m_blescan_info_pri.item_cnt);
                                if(old_adv_state)
                                {
                                    old_adv_state = false;
                                    cfg_ble_advertising_start_stop_req(true);
                                }
                                if(m_blescan_info.item_cnt > 0)
                                {
                                    bScaned = true;
                                }

                                if(bScaned)
                                {
                                    int8_t rssi_s8 = 0;
                                    int i;
                                    uint32_t idx;
                                    bool pos_data_flag = false;
                                    uint8_t cur_mac_buf[6];
                                    uint8_t *p_addr;
                                    uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};

                                    if(m_blescan_info_pri.item_cnt > 0)
                                    {
                                        rssi_s8 = m_blescan_info_pri.adv_info[0].rssi;
                                        //search hightest rssi item
                                        for(i = 0; i < m_blescan_info_pri.item_cnt; i++)
                                        {
                                            if(rssi_s8 <= m_blescan_info_pri.adv_info[i].rssi)
                                            {
                                                p_addr = m_blescan_info_pri.adv_info[i].peer_addr.addr;
                                                //reverse mac addr
                                                cur_mac_buf[0] = p_addr[5];
                                                cur_mac_buf[1] = p_addr[4];
                                                cur_mac_buf[2] = p_addr[3];
                                                cur_mac_buf[3] = p_addr[2];
                                                cur_mac_buf[4] = p_addr[1];
                                                cur_mac_buf[5] = p_addr[0];
                                                rssi_s8 = m_blescan_info_pri.adv_info[i].rssi;
                                            }
                                        }
                                        //get pos data
                                        for(i = 0; i < m_registered_mac_info.ble_mac_cnt; i++)
                                        {
                                            if((memcmp(m_registered_mac_info.ble_r_mac[i], cur_mac_buf, 6) == 0)
                                                && (m_registered_mac_info.ble_r_position[i][0] != 0xff /*empty value*/ ))
                                            {
                                                pos_data_flag = true;
                                                idx = (uint32_t)i;
                                                break;
                                            }
                                        }
                                        m_module_tracking_data.ble_scan_status = true;
                                        m_module_tracking_data.ble_scan_data_cnt = 1;
                                        memcpy(m_module_tracking_data.ble_scan_data, cur_mac_buf, 6);
                                        m_module_tracking_data.ble_scan_rssi[0]=(int8_t)rssi_s8;
                                        cPrintLog(CDBG_FCTRL_INFO, "BLE Scan OK! posFlag:%d Mac:%02x%02x%02x%02x%02x%02x RSSI:%d\n", pos_data_flag, cur_mac_buf[0],cur_mac_buf[1], cur_mac_buf[2], cur_mac_buf[3], cur_mac_buf[4], cur_mac_buf[5], (int)rssi_s8);
                                    }

                                    if(pos_data_flag)
                                    {
                                        m_module_tracking_data.ble_scan_match_gps_status = true;
                                        memcpy(m_module_tracking_data.ble_scan_match_gps_data, m_registered_mac_info.ble_r_position[idx], sizeof(m_module_tracking_data.ble_scan_match_gps_data));
                                        cache_gps_data((uint8_t *)m_registered_mac_info.ble_r_position[idx]);  //print pos_data and LatitudeLongitude
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                                        encode_12byte_dataV2_PosData(main_send_data_ble_pos, send_data, main_wakeup_reason, (uint8_t *)m_registered_mac_info.ble_r_position[idx], batt_avg_report_percent, magnet_status);
#else
                                        encode_12byte_dataV2_PosData(main_send_data_ble_pos, send_data, main_wakeup_reason, (uint8_t *)m_registered_mac_info.ble_r_position[idx], batt_avg_report_percent, tmp108_data);
#endif
                                        cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                                        cPrintLog(CDBG_FCTRL_INFO, "BLE_SCAN make data:%s\n", frame_data);
                                    }
                                    else
                                    {
//                                       if(cfg_peripheral_device_enable_status_get(PTYPE_WIFI))
//                                       {
//                                            bScaned = false;
//                                       }
//                                       else
                                        {
                                            //send ble data
                                            if(bScaned)
                                            {
                                                p_addr = m_blescan_info.adv_info[0].peer_addr.addr;
                                                //reverse mac addr
                                                cur_mac_buf[0] = p_addr[5];
                                                cur_mac_buf[1] = p_addr[4];
                                                cur_mac_buf[2] = p_addr[3];
                                                cur_mac_buf[3] = p_addr[2];
                                                cur_mac_buf[4] = p_addr[1];
                                                cur_mac_buf[5] = p_addr[0];
                                                
                                                m_module_tracking_data.ble_scan_status = true;
                                                m_module_tracking_data.ble_scan_data_cnt = 1;
                                                memcpy(m_module_tracking_data.ble_scan_data, cur_mac_buf, 6);
                                                m_module_tracking_data.ble_scan_rssi[0]=m_blescan_info.adv_info[0].rssi;
                                                cPrintLog(CDBG_FCTRL_INFO, "Ble Scan Data!  Mac:%02x%02x%02x%02x%02x%02x RSSI:%d\n", cur_mac_buf[0],cur_mac_buf[1], cur_mac_buf[2], cur_mac_buf[3], cur_mac_buf[4], cur_mac_buf[5], (int)m_blescan_info.adv_info[0].rssi);
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                                                encode_12byte_dataV2_BLE(send_data, main_wakeup_reason, cur_mac_buf, m_blescan_info.adv_info[0].rssi, batt_avg_report_percent, magnet_status);  //use mac
#else
                                                encode_12byte_dataV2_BLE(send_data, main_wakeup_reason, cur_mac_buf, m_blescan_info.adv_info[0].rssi, batt_avg_report_percent, tmp108_data);  //use mac
#endif
                                                cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);

                                            }
                                            else
                                            {
                                                //No data
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                                                encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, magnet_status);
#else
                                                encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, tmp108_data);
#endif
                                                cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                                            }
                                        }
                                    }
                                }
                                ble_scan_go_next_state = true;
                            }
                        }
                    }
                    ++m_module_ready_wait_timeout_tick;
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "ble scan disabled!\n");
                    ble_scan_go_next_state = true;
                }
#else
                cPrintLog(CDBG_FCTRL_INFO, "CDEV_BLE_SCAN_ENABLE not enabled!\n");
                ble_scan_go_next_state = true;
#endif
                if(ble_scan_go_next_state)
                {
                    main_set_module_state(main_get_module_state_next(bScaned));
                }
            }
            break;

        case WIFI:
            {
                bool wifi_go_next_state = false;
                uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE];
                uint32_t get_cnt = 0;
                uint8_t *bssidBuf;
                int32_t *rssi;
                uint8_t bssidBuf_ssidlist[6*2];
                int32_t rssi_ssidlist[2];

#ifdef CDEV_WIFI_MODULE
                int wifi_result;
                int get_bssid_result;
                if(cfg_peripheral_device_enable_status_get(PTYPE_WIFI))
                {
                    if(m_init_excute)
                    {
#ifdef FEATURE_CFG_BLE_UART_CONTROL
                        memset(m_nus_service_parameter.wifi_data, 0, sizeof(m_nus_service_parameter.wifi_data));
                        memset(m_nus_service_parameter.wifi_rssi, 0, sizeof(m_nus_service_parameter.wifi_rssi));
#endif
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
#if 0  //support zone change
                        {
                            int txIdx = 0;  /*0:FCC, 1:CE, 2: TELEC*/
                            CWIFI_CHANNEL_TYPE_e channelType = CWIFI_CHANNEL_TYPE_DEFAULT;
                            const uint8_t txPowerTable[3][6] = {
                                {0x3A, 0x3A, 0x3A, 0x3A, 0x32, 0x2E},  //FCC default
                                {0x3B, 0x3B, 0x3B, 0x3B, 0x38, 0x38},  //CE default
                                {0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C}   //Telec default
                            };

                            //1=RC1, 2=RC2, 3=RC3c, 4=RC4, 5=RC5, 6=RC6
                            if(m_module_parameter.sigfox_RC_number == 1)
                                txIdx = 1;
                            else if(m_module_parameter.sigfox_RC_number == 2)
                                txIdx = 0;
                            else if(m_module_parameter.sigfox_RC_number == 3)
                                txIdx = 2;
                            else if(m_module_parameter.sigfox_RC_number == 4)
                                txIdx = 0;
                            else if(m_module_parameter.sigfox_RC_number == 5)
                                txIdx = 1;
                            else
                                txIdx = 0; //fcc is default

                            if(txIdx ==0)  //FCC
                                channelType = CWIFI_CHANNEL_TYPE_12;
                            else if(txIdx ==1)  //CE
                                channelType = CWIFI_CHANNEL_TYPE_13;
                            else if(txIdx ==2)  //TELECT
                                channelType = CWIFI_CHANNEL_TYPE_14;
                            else
                                channelType = CWIFI_CHANNEL_TYPE_DEFAULT;

                            cWifi_Set_Scan_Type(CWIFI_SCAN_TYPE_PASSIVE);
                            cWifi_Set_Channel_Type(channelType);
                            cWifi_Check_N_Update_TxPower(txPowerTable[txIdx]);
                        }
#else
                        cWifi_Set_Scan_Type(CWIFI_SCAN_TYPE_PASSIVE);
#endif
#else
                        cWifi_Set_Scan_Type(CWIFI_SCAN_TYPE_ACTIVE);
#endif
                        if(m_registered_mac_info.wifi_mac_cnt > 0)
                        {
                            wifi_result = cWifi_ap_scan_req_priority_mac_list(m_registered_mac_info.wifi_mac_cnt, (uint8_t *)m_registered_mac_info.wifi_r_mac);  //scan priority registred mac scan request
                        }
                        else
                        {
                            if(m_ssid_list_info.ssid_white_cnt > 0 || m_ssid_list_info.ssid_black_cnt > 0)
                            {
                                cWifi_SetSsidList(m_ssid_list_info.ssid_white_cnt, (const uint8_t *)m_ssid_list_info.ssid_white_list, m_ssid_list_info.ssid_black_cnt, (const uint8_t *)m_ssid_list_info.ssid_black_list);
                            }
                            wifi_result = cWifi_ap_scan_req();  //scan request (normal scan)
                        }

                        if(wifi_result == CWIFI_Result_OK)
                        {
                            cTBC_write_state_noti("WifiScan");
                            cPrintLog(CDBG_FCTRL_INFO, "WIFI Start\n");
                            m_init_excute = false;
                        }
                        else
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module! send NUll data!\n");
                            wifi_go_next_state = true;
                        }
                    }
                    else
                    {
                        if(!cWifi_is_scan_state() && !cWifi_bus_busy_check())  //wait scan
                        {
                            get_bssid_result = CWIFI_Result_NoData;
                            if(m_ssid_list_info.ssid_white_cnt > 0 || m_ssid_list_info.ssid_black_cnt > 0)
                            {
                                uint32_t getIdx;
                                uint32_t getCnt = 0, getCntWhitelist = 0, getCntHiddenssid = 0;
                                uint8_t *getSsid, *getSsidWhitelist, *getSsidHiddenssid;
                                int32_t *getRssi, *getRssiWhitelist, *getRssiHiddenssid;
                                uint8_t *getBssid, *getBssidWhitelist, *getBssidHiddenssid;

                                cWifi_get_scan_result(&getCnt, &getSsid, &getRssi, &getBssid);
                                cWifi_get_scan_result_whitelist(&getCntWhitelist, &getSsidWhitelist, &getRssiWhitelist, &getBssidWhitelist);
                                cWifi_get_scan_result_hiddenssid(&getCntHiddenssid, &getSsidHiddenssid, &getRssiHiddenssid, &getBssidHiddenssid);

                                get_cnt = 0;
                                memset(bssidBuf_ssidlist, 0, sizeof(bssidBuf_ssidlist));
                                bssidBuf = bssidBuf_ssidlist;
                                memset(rssi_ssidlist, 0, sizeof(rssi_ssidlist));
                                rssi = rssi_ssidlist;
                                cPrintLog(CDBG_FCTRL_INFO, "wifi ssidlist scan result  : %d, %d, %d\n", getCnt, getCntWhitelist, getCntHiddenssid);
                                if(getCntWhitelist > 0)
                                {
                                    for(getIdx = 0; getIdx < getCntWhitelist; getIdx++)
                                    {
                                        if(get_cnt < 2)
                                        {
                                            memcpy(&bssidBuf[CWIFI_BSSID_SIZE*get_cnt], &getBssidWhitelist[CWIFI_BSSID_SIZE*getIdx], CWIFI_BSSID_SIZE);
                                            rssi[get_cnt] = getRssiWhitelist[getIdx];
                                            get_cnt++;
                                        }
                                    }
                                }
                                if(getCnt > 0)
                                {
                                    for(getIdx = 0; getIdx < getCnt; getIdx++)
                                    {
                                        if(get_cnt < 2)
                                        {
                                            memcpy(&bssidBuf[CWIFI_BSSID_SIZE*get_cnt], &getBssid[CWIFI_BSSID_SIZE*getIdx], CWIFI_BSSID_SIZE);
                                            rssi[get_cnt] = getRssi[getIdx];
                                            get_cnt++;
                                        }
                                    }
                                }
                                if(get_cnt > 0)
                                    get_bssid_result = CWIFI_Result_OK;
                                else
                                    get_bssid_result = CWIFI_Result_NoData;
                            }
                            else
                            {
                                get_bssid_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
                                if(get_bssid_result == CWIFI_Result_OK)
                                {
                                    cWifi_get_scan_result(&get_cnt, NULL, &rssi, NULL);
                                    cPrintLog(CDBG_FCTRL_INFO, "wifi scan result  : %d\n", get_cnt);
#ifdef FEATURE_CFG_BLE_UART_CONTROL
                                    memcpy(m_nus_service_parameter.wifi_data, bssidBuf, sizeof(m_nus_service_parameter.wifi_data));
                                    m_nus_service_parameter.wifi_rssi[0] = (int8_t)rssi[0];
                                    m_nus_service_parameter.wifi_rssi[1] = (int8_t)rssi[1];
                                    m_nus_service_parameter.module = 'W';
                                    nus_send_data('W');
#endif
                                }
                                else if(get_bssid_result == CWIFI_Result_NoData)
                                {
                                    cPrintLog(CDBG_FCTRL_INFO, "WIFI NoData! send NUll data!\n");
                                }
                                else
                                {
                                    cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module! send NUll data!\n");
                                }
                            }
                            if(get_bssid_result == CWIFI_Result_OK)
                            {
                            }
                            else if(get_bssid_result == CWIFI_Result_NoData)
                            {
                                cPrintLog(CDBG_FCTRL_INFO, "WIFI NoData! send NUll data!\n");
                            }
                            else
                            {
                                cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module! send NUll data!\n");
                            }
                            wifi_go_next_state = true;
                        }
                    }
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "WIFI disabled!\n");
                    wifi_go_next_state = true;
                }
#else
                cPrintLog(CDBG_FCTRL_INFO, "CDEV_WIFI_MODULE Not Defined!\n");
                wifi_go_next_state = true;
#endif
                if(wifi_go_next_state)
                {
                    memset(send_data, 0, sizeof(send_data));
                    if(get_cnt > 0)
                    {
                        int8_t rssi_s8 = 0;
                        int i, idx;
                        bool pos_data_flag = false;

                        for(i = 0; i < m_registered_mac_info.wifi_mac_cnt; i++)
                        {
                            if((memcmp(m_registered_mac_info.wifi_r_mac[i], bssidBuf, 6) == 0)
                                && (m_registered_mac_info.wifi_r_position[i][0] != 0xff /*empty value*/ ))
                            {
                                pos_data_flag = true;
                                idx = i;
                                break;
                            }
                        }

                        if(rssi)rssi_s8=(uint8_t)rssi[0];
                        m_module_tracking_data.wifi_status = true;
                        m_module_tracking_data.wifi_data_cnt = 1;
                        memcpy(m_module_tracking_data.wifi_data, bssidBuf, 6);
                        m_module_tracking_data.wifi_rssi[0]=(int8_t)rssi_s8;
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Scan OK! posFlag:%d, Mac:%02x%02x%02x%02x%02x%02x RSSI:%d\n", pos_data_flag, bssidBuf[0], bssidBuf[1], bssidBuf[2], bssidBuf[3], bssidBuf[4], bssidBuf[5], (int)rssi_s8);

                        if(pos_data_flag)
                        {
                            m_module_tracking_data.wifi_match_gps_status = true;
                            memcpy(m_module_tracking_data.wifi_match_gps_data, m_registered_mac_info.wifi_r_position[idx], sizeof(m_module_tracking_data.wifi_match_gps_data));
                            cache_gps_data((uint8_t *)m_registered_mac_info.wifi_r_position[idx]);  //print pos_data and LatitudeLongitude
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                            encode_12byte_dataV2_PosData(main_send_data_wifi_pos, send_data, main_wakeup_reason, (uint8_t *)m_registered_mac_info.wifi_r_position[idx], batt_avg_report_percent, magnet_status);
#else
                            if(get_cnt == 1)
                                memcpy(send_data, bssidBuf, 6);
                            else
                                memcpy(send_data, bssidBuf, 12);
#endif
                        }
                        else
                        {
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                            if(get_cnt == 1)
                            {
                                encode_12byte_dataV2_WIFI(send_data, main_wakeup_reason, bssidBuf, rssi_s8, batt_avg_report_percent, magnet_status);  //use mac
                            }
                            else
                            {
                                memcpy(send_data, bssidBuf, 12);
                            }
#else
                            if(get_cnt == 1)
                                memcpy(send_data, bssidBuf, 6);
                            else
                                memcpy(send_data, bssidBuf, 12);
#endif
                        }
                        cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI make data:%s\n", frame_data);
                        main_set_module_state(main_get_module_state_next(true));
                    }
                    else
                    {
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
                        encode_12byte_dataV2_NoTracking(send_data, main_wakeup_reason, batt_avg_report_percent, magnet_status);
#endif
                        cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Null Data!:%s\n", frame_data);
                        main_set_module_state(main_get_module_state_next(false));
                    }
                    
                }
            }
            break;

        case SIGFOX:
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
            if(sigfox_get_state() == SETUP_S)
            {
                if(m_init_excute)
                {
                    cTBC_write_state_noti("Sigfox");
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX Start RC:%d DL:%d\n", m_module_parameter.sigfox_RC_number, m_module_parameter.sigfox_recv_en);
#else
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX Start DL:%d\n", m_module_parameter.sigfox_recv_en);
#endif
                    if(test_nus_full_tracking_mode)
                    {
                        module_make_cached_tracking_data();
                    }
                    else
                    {
                        if(m_module_parameter.operation_mode == 0/*station mode*/)
                        {
                            cPrintLog(CDBG_FCTRL_INFO, "make sensor data for station mode!\n");
                            module_make_sensor_data();
                        }
                        else if(m_module_parameter.operation_mode == 2/*full tracking mode*/)  //It is for engineer mode. replace payload
                        {
                            module_make_cached_tracking_data();
                        }
                    }
                    cfg_sigfox_timers_start();
                    m_init_excute = false;
                }
            }
            else if(sigfox_check_exit_excute())
            {
                if(m_sigfox_next_state == NONE)
                {
                    main_set_module_state(main_get_module_state_next(true));
                }
                else
                {
                    main_set_module_state(m_sigfox_next_state);
                    m_sigfox_next_state = NONE;
                }
                cfg_sigfox_timers_stop();
                sigfox_set_state(SETUP_S);
                nus_send_data('T');
                nus_send_data('B');
            }
#else
            cPrintLog(CDBG_FCTRL_INFO, "Data Modem Not defined! goto next state\n");
            main_set_module_state(main_get_module_state_next(true));
#endif

            break;

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
        case SIGFOX_CHECK_RUN_SCAN_RC:
            if(m_module_parameter.sigfox_scan_rc_mode != RC_SCAN_DISABLE)
            {
                cPrintLog(CDBG_FCTRL_INFO, "RC SCAN mode : %d\n", m_module_parameter.sigfox_scan_rc_mode);
                main_set_module_state(main_get_module_state_next(true));
            }
            else
            {
                cPrintLog(CDBG_FCTRL_INFO, "RC SCAN disabled! Sigfox Send Req! [Zone:%d]\n", m_module_parameter.sigfox_RC_number);
                main_set_module_state(main_get_module_state_next(false));
            }
            break;
        case SIGFOX_SCAN_RC:
            if(sigfox_get_state() == SETUP_S)
            {
                if(m_init_excute)
                {
                    scan_rc_parameter = true;
                    sigfox_rc_checked  = false;
                    cTBC_write_state_noti("SigfoxScanRC");
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan Start\n");
                    cfg_sigfox_timers_start();
                    m_init_excute = false;
                }
            }
            else if(sigfox_check_exit_excute())
            {
                scan_rc_parameter = false;
                cfg_sigfox_timers_stop();
                sigfox_set_state(SETUP_S);
                if(sigfox_rc_checked)  //rc scaned
                {
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan OK! [Zone:%d]\n", m_module_parameter.sigfox_RC_number);
                    main_set_module_state(main_get_module_state_next(true));
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan Failed!\n");
                    if(m_module_parameter.sigfox_scan_rc_mode == RC_SCAN_ALWAYS)
                    {
                        main_set_module_state(main_get_module_state_next(true));
                    }
                    else if(m_module_parameter.sigfox_scan_rc_mode == RC_SCAN_ONLY_SEND_WHEN_CONFIRMED)
                    {
                        main_set_module_state(main_get_module_state_next(false));
                    }
                    else
                    {
                        main_set_module_state(main_get_module_state_next(false));
                    }
                }
            }
            break;
#endif

        case IDLE:
            cPrintLog(CDBG_FCTRL_INFO, "Wakeup[%d][%d]\n", main_wakeup_reason,test_time_sec);
            m_wait_retry_event = false;
            main_timer_schedule_stop();
            main_timer_schedule_start();
            main_set_module_state(main_get_module_state_next(true));
            set_wait_time_state();
            break;
        case WAIT:
            if(main_wakeup_reason == main_wakeup_reason_magnetic_event_det)
            {
                main_set_module_state(main_get_module_state_next(true));
            }
            else if(main_wakeup_reason == main_wakeup_reason_acc_event_motion)
            {
                cfg_sigfox_downlink_on_off(true);
                main_set_module_state(main_get_module_state_next(true));
            }
            else if(test_nus_full_tracking_mode)
            {
                main_set_module_state(main_get_module_state_next(true));
                m_wait_retry_sending= false;
            }
            else
            {
                if(m_wait_retry_sending)
                {
    //              if(get_no_motion_state())
                    {
                    m_wait_retry_sending= false;
                    cPrintLog(CDBG_FCTRL_INFO, "Start sending[%d]\n", test_time_sec);
                    main_set_module_state(main_get_module_state_next(true));
                    }
                }
                else
                {
                    if(get_no_motion_state()||get_wait_time_state())
                    {
    //                      m_wait_retry_event = get_wait_time_state();
                        if(downlink_count >= downlink_max)
                        {
                            cfg_sigfox_downlink_on_off(true);
                            downlink_count = 1;
                        }
                        else
                        {
                            downlink_count++;
                            cfg_sigfox_downlink_on_off(false);
                        }
                        cPrintLog(CDBG_FCTRL_INFO, "Start sending[%d]\n", test_time_sec);
                        main_set_module_state(main_get_module_state_next(true));
                    }
                }
            }
            break;
        default:
            break;
    }
}

static void main_bypass_enter_CB(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "call %s\n", __func__);
    cfg_ble_led_control(false);
#ifdef CDEV_WIFI_MODULE
    if(!cfg_peripheral_device_enable_status_get(PTYPE_WIFI))
    {
        cWifi_prepare_start(m_module_peripheral_ID.wifi_MAC_STA);
    }
#endif

#ifdef CDEV_GPS_MODULE
    if(!cfg_peripheral_device_enable_status_get(PTYPE_GPS))
    {
        cGps_resource_init();
        cGps_prepare_start();
    }
#endif
}

static void main_bypass_exit_CB(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "call %s\n", __func__);
    cfg_ble_led_control(true);
}

/**
 * @brief function for creating main schedule timer
 */

static void main_timer_create(void)
{
    app_timer_timeout_handler_t timeout_handler;
    volatile uint32_t      err_code;

    timeout_handler = main_schedule_timeout_handler_asset_tracker;   
    err_code = app_timer_create(&m_main_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief function for enter and wakeup deepsleep
 */
static void main_deepsleep_control(void)  //enter deepsleep at booting
{
#ifdef BD_FEATURE_POWEROFF_AT_FIRSTTIME
    bool old_nfc_tag_on;
    bool force_wakeup = 0;
    cPrintLog(CDBG_MAIN_LOG, "wait boot unlock\n");

#if defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY) && defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON)
    bool wkupdet = false;
    nrf_gpio_cfg_input(USR_MODULE_GPIO_DEF_WAKEUP_KEY, NRF_GPIO_PIN_PULLDOWN);
#endif
#if defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) && defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON)
    bool magnetdet = false;
    nrf_gpio_cfg_input(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL, NRF_GPIO_PIN_NOPULL);
#endif

    if(0){} //dummy if for ifdef
#if defined(BD_FEATURE_POWEROFF_AT_FIRSTTIME_WHEN_I2CDBG_NOT_CONNCETED)
    else if(cfg_board_check_bootstrap_checked_i2cdbg_connected())
    {
        force_wakeup = 1;
    }
#endif

    for (;; )
    {
#if defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY) && defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON)
        wkupdet = (nrf_gpio_pin_read(USR_MODULE_GPIO_DEF_WAKEUP_KEY) == 1)?1:0;
#endif
#if defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) && defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON)
        magnetdet = (nrf_gpio_pin_read(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) == USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_ACTIVE_LEVEL)?1:0;
#endif
        if (m_nfc_tag_on  //wake up condition /*or m_cfg_NFC_wake_up_detected*/
            || m_cfg_sw_reset_detected  //wake up condition
#if defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY) && defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON)
            || wkupdet  //wake up condition
#endif
#if defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) && defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON)
            || magnetdet  //wake up condition
#endif
            || m_cfg_GPIO_wake_up_detected  //wake up condition
            || m_cfg_debug_interface_wake_up_detected
            || cTBC_check_host_connected()  //exception condition
            || force_wakeup
         )
        {
            old_nfc_tag_on = m_nfc_tag_on;
            m_nfc_tag_on = false;
            cPrintLog(CDBG_MAIN_LOG, "wakeup reason swreset:%d, gpio:%d, nfc:%d %d, jtag:%d\n", 
                          m_cfg_sw_reset_detected, m_cfg_GPIO_wake_up_detected, m_cfg_NFC_wake_up_detected, old_nfc_tag_on, m_cfg_debug_interface_wake_up_detected);
#if defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY) && defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON)
            cPrintLog(CDBG_MAIN_LOG, "wkupdet:%d\n", wkupdet);
            nrf_gpio_cfg_default(USR_MODULE_GPIO_DEF_WAKEUP_KEY);
#endif
#if defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) && defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON)
            cPrintLog(CDBG_MAIN_LOG, "magnetdet:%d\n", magnetdet);
            nrf_gpio_cfg_default(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL);
#endif
            cPrintLog(CDBG_MAIN_LOG, "force wakeup:%d\n", force_wakeup);
            return;
        }
        if(cTBC_is_busy())
        {
            cfg_board_power_manage();  //The wakeup event is generated by the cfg_twis_board_control
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "enter deep sleep mode! swreset:%d, gpio:%d, nfc:%d, jtag:%d\n", m_cfg_sw_reset_detected, m_cfg_GPIO_wake_up_detected, m_cfg_NFC_wake_up_detected, m_cfg_debug_interface_wake_up_detected);
            cfg_board_prepare_power_down();
            cfg_board_goto_power_down();
        }
    }
#endif
}


void nfc_tag_on_callback(void)
{
    cTBC_event_noti("NFC_TAG");
}

void main_send_poweroff_msg(void)
{
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
    uint8_t send_msg_data[SIGFOX_SEND_PAYLOAD_SIZE];
    encode_12byte_dataV2_Sensor(send_msg_data, main_wakeup_reason_powerdown, &m_accel, als_lux, batt_avg_report_percent, tmp108_data);
    cfg_ble_led_control(true);
    sigfox_send_payload(send_msg_data, SIGFOX_SEND_PAYLOAD_SIZE, NULL);
#endif
}

static void tbc_over_rtt_sec_tick_proc(void)
{
    main_Sec_tick++;
    motion_sec_tic++;
    no_motion_sec_tic++;
    wait_time_sec_tic++;
    test_time_sec++;
//    main_GPS_BKUP_check_handler();
//  set_no_motion_state();
}

void main_test_for_sleep_tick(void)
{
#if 1  //test for led blink
    static int timer_tick = 0;
    if(++timer_tick % 10 == 0)
    {
        cfg_ble_led_control(true);
    }
    else
    {
        cfg_ble_led_control(false);
    }
#endif
}

void main_test_for_peripherals_current_consumption(void)
{
    //Insert the code that you want to measure the current consumption.
}

void cfg_downlink_max_set()
{
    downlink_max = (CFG_ONE_DAY_SEC / m_module_parameter.idle_time)*m_module_parameter.downlink_day;
}
static void printout_ID_Info(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "BLE MAC:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.ble_MAC, 6);
#ifdef CDEV_SIGFOX_MODULE
    cPrintLog(CDBG_FCTRL_INFO, "SFX ID:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.sigfox_device_ID, 4);
    cPrintLog(CDBG_FCTRL_INFO, "SFX PAC CODE:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.sigfox_pac_code, 8);
#endif
#ifdef CDEV_WIFI_MODULE
    cPrintLog(CDBG_FCTRL_INFO, "WIFI MAC:");
    cDataDumpPrintOut(CDBG_FCTRL_INFO, m_module_peripheral_ID.wifi_MAC_STA, 6);  
#endif
}

static void printout_WIFI_Info(void)
{
#ifdef CDEV_WIFI_MODULE
    uint8_t wifi_app_ver;
    uint16_t initDataVer;
    
    cWifi_get_version_info(&wifi_app_ver, &initDataVer);
    cPrintLog(CDBG_FCTRL_INFO, "WIFI AppVer:%02x, InitDataVer:%04x\n", wifi_app_ver, initDataVer);
#endif
}

static void printout_operation_mode_Info(void)
{
    if(m_module_parameter.operation_mode == 0)
    {
        cPrintLog(CDBG_FCTRL_INFO, "station mode\n");
    }
    else if(m_module_parameter.operation_mode == 1)
    {
        cPrintLog(CDBG_FCTRL_INFO, "smart tracking mode:");
    }
    else if(m_module_parameter.operation_mode == 2)
    {
        cPrintLog(CDBG_FCTRL_INFO, "full tracking mode[It is for engineer mode.]");
    }
    else
    {
        cPrintLog(CDBG_FCTRL_INFO, "unknown operation mode\n");
    }

    if(m_module_parameter.operation_mode)
    {
        cPrintLog(CDBG_FCTRL_INFO, "[GPS:%d, WIFI:%d, BLE:%d, ACC:%d, MAG:%d, TMP:%d, Light:%d]\n"
            , m_module_parameter.gps_enable
            , m_module_parameter.wifi_enable
            , m_module_parameter.ble_beacon_scan_enable
            , m_module_parameter.acc_operation_mode
            , m_module_parameter.magnet_operation_mode
            , m_module_parameter.temperature_sensor_enable
            , m_module_parameter.ambient_light_sensor_enable);
    }
}

static void printout_main_info(void)
{
    printout_operation_mode_Info();
    printout_ID_Info();
    printout_WIFI_Info();
}

void main_external_io_devices_init(void)
{
#ifdef USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
    if(m_module_parameter.magnet_operation_mode)
        cfg_magnetic_sensor_init(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_ACTIVE_LEVEL, USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL, main_magnet_attach_callback, main_magnet_detach_callback);
#endif

#ifdef USR_MODULE_GPIO_DEF_WAKEUP_KEY
    cfg_wkup_gpio_init(USR_MODULE_GPIO_DEF_WAKEUP_KEY, main_wkup_det_callback);
#endif

#ifdef USR_MODULE_GPIO_DEF_BUTTON
    cfg_button_init(USR_MODULE_GPIO_DEF_BUTTON_ACTIVE_LEVEL, USR_MODULE_GPIO_DEF_BUTTON, main_button_short_press_callback, main_button_long_press_callback);
#endif

#ifdef USR_MODULE_DEF_BATTERY_ADC_INPUT
    cfg_battery_check_init(USR_MODULE_DEF_BATTERY_ADC_INPUT);
#endif
}

void main_external_i2c_devices_init(void)
{
    if(cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE) 
        && cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
    {
        cfg_tmp102_timer_create();
        cfg_opt3001_timer_create();
        if(tmp102_get_enable_intr() && opt3001_get_enable_intr())
        {
            cfg_tmp10xopt3001_interrupt_init();
        }
        else if(tmp102_get_enable_intr())
        {
            cfg_tmp10x_interrupt_init();
        }
        else if(opt3001_get_enable_intr())
        {
            cfg_opt3001_interrupt_init();
        }
    }
    else if(cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE))
    {
        cfg_tmp102_timer_create();
        if(tmp102_get_enable_intr())
        {
            cfg_tmp10x_interrupt_init();
        }
#ifdef CDEV_AMBIENT_LIGHT_SENSOR
        opt3001_set_shutdown();
#endif
    }
    else if(cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
    {
        cfg_opt3001_timer_create();
        if(opt3001_get_enable_intr())
        {
            cfg_opt3001_interrupt_init();
        }
        tmp102_req_shutdown_mode();
    }
    else if(!cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE) 
        && !cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
    {
#ifdef CDEV_AMBIENT_LIGHT_SENSOR
        opt3001_set_shutdown();
#endif

#if defined(CDEV_TEMPERATURE_SENSOR_TMP102) || defined(CDEV_TEMPERATURE_SENSOR_TMP108)
        tmp102_req_shutdown_mode();
#endif
    }

}

void main_ble_init(void)
{
    advertising_info_init();
    cfg_ble_peer_manager_init(m_module_parameter_rebuiled_flag);
    cfg_ble_gatt_init();
    cfg_ble_services_init(m_module_parameter.fota_enable
#ifdef FEATURE_CFG_BLE_UART_CONTROL
                          , true, nus_recv_data_handler
#else
                          , false, NULL
#endif
                          );
    cfg_ble_advertising_init(MSEC_TO_UNITS(m_module_parameter.beacon_interval, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, on_cBle_adv_evt);
    cfg_ble_conn_params_init();
}

void main_scheduler_init(void)
{
    main_timer_create();
}

void main_basic_resource_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    cfg_ble_stack_init(on_cBle_evt);
    sd_power_dcdc_mode_set(1);

    cfg_board_pwr_mgmt_init();
}

void i2c_device_enter_deepsleep(void)
{
    nrf_delay_ms(1);
    cfg_i2c_master_send_General_Call_Reset();
    nrf_delay_ms(1);
#ifndef CDEV_DISABLE_ACC_MODULE_LIB
#ifdef CDEV_ACC_MODULE
    cfg_bma250_req_suppend_mode();
    nrf_delay_ms(1);
#endif
#endif

#if defined(CDEV_TEMPERATURE_SENSOR_TMP102) || defined(CDEV_TEMPERATURE_SENSOR_TMP108)
    tmp102_req_shutdown_mode();
    nrf_delay_ms(1);
#endif

#ifdef CDEV_AMBIENT_LIGHT_SENSOR
    opt3001_set_shutdown();
    nrf_delay_ms(1);
#endif
}
int main(void)
{
    ret_code_t err_code;
    cPrintLog(CDBG_MAIN_LOG, "\n====== %s Module Started Ver:%s bdtype:%d======\n", m_cfg_model_name, m_cfg_sw_ver, m_cfg_board_type);
    cPrintLog(CDBG_MAIN_LOG, "build date:%s, %s\n", m_cfg_build_date, m_cfg_build_time);

    //start of early init ////////
    module_parameter_early_read(); //just read only for module parameter
    m_cfg_board_shutdown_i2c_dev_func = i2c_device_enter_deepsleep;
    cfg_board_early_init(cfg_board_prepare_power_down);

//test_for_low_current_consumption
//    cfg_board_testmode_BLE_Advertising_LowPwr(true, false, main_test_for_peripherals_current_consumption, main_test_for_sleep_tick);

    main_basic_resource_init();
    cfg_scen_init();
    cfg_nvm_init();
    cfg_downlink_max_set();

#ifdef FEATURE_CFG_BLE_UART_CONTROL
    nus_data_init();
#endif

#ifdef FEATURE_CFG_USE_I2C0_DBG_PIN
    cTBC_init(dbg_i2c_user_cmd_proc, true);  //use i2c debug pin
#else
    cTBC_init(dbg_i2c_user_cmd_proc, false);  //not use i2c debug pin
#endif
    cTBC_OVER_RTT_init(tbc_over_rtt_sec_tick_proc); //depend on cTBC_init() //FEATURE_CFG_RTT_MODULE_CONTROL
    cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);

#ifdef USR_MODULE_FUNCTION_USE_NFC
    cfg_nfc_init(nfc_tag_on_callback);
#endif

    main_deepsleep_control();
    cfg_board_gpio_set_default_gps();
    cfg_ble_led_control(true);
    cfg_board_init();
    printout_main_info();
    if(module_parameter_ID_value_update())
    {
#ifdef USR_MODULE_FUNCTION_USE_NFC
        cfg_nfc_restart();
#endif
    }
    main_scheduler_init();
    main_external_io_devices_init();
    main_external_i2c_devices_init();
    main_ble_init();
    
    cTBC_check_N_enter_bypassmode(200, main_bypass_enter_CB, main_bypass_exit_CB);
    if(m_hitrun_test_flag)cfg_board_reset();

//test_for_low_current_consumption
//    cfg_board_testmode_BLE_Advertising_LowPwr(false, false, main_test_for_peripherals_current_consumption, main_test_for_sleep_tick);

    // Start execution.
    cPrintLog(CDBG_FCTRL_INFO, "main schedule start! state:%d\n", m_module_mode);
    main_wakeup_reason = main_wakeup_reason_powerup;
    cfg_ble_advertising_start_stop_req(true);
    main_timer_schedule_start();
    m_module_state_started = true;

    for (;; )
    {
        cfg_scen_main_handler();
#ifdef USR_MODULE_FUNCTION_USE_NFC
        cfg_nfc_main_handler();
#endif
        cfg_board_power_manage();
    }
}


/**
 * @}
 */
