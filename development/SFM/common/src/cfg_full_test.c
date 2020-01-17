/* Copyright (c) 2018 ieTings Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @brief tracking Sample Application cfg_full_test.c file.
 *
 * This file contains the source code for an tracking sample application.
 */


#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "nrf_queue.h"
#include "sdk_macros.h"
#include "ble_advertising.h"
#include "app_timer.h"

#include "cfg_config.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_adc_battery_check.h"
#include "cfg_scenario.h"
#include "cfg_board.h"
#include "cfg_sigfox_module.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_ble_ctrl.h"
#include "cfg_dbg_log.h"
#include "cfg_bma250_module.h"
#include "cfg_full_test.h"
#include "cfg_nus_cmd_proc.h"

#ifdef FEATURE_CFG_FULL_TEST_CMD_WITH_BLE_NUS
typedef enum
{
    FULL_TEST_NONE_S,
    FULL_TEST_INIT_S,
    FULL_TEST_FULL_TEST_START_S,
    FULL_TEST_BATTERY_CHECK_S,
    FULL_TEST_GPS_S,
    FULL_TEST_WIFI_S,
    FULL_TEST_SIGFOX_S,
    FULL_TEST_IDLE_SLEEP_S,
    FULL_TEST_STATE_MAX_S
}full_test_scheduler_state_e;

static uint32_t m_full_test_mode_tick = 0;
static bool m_full_test_ble_connect_flag = false;
static bool m_full_test_running_flag = false;
static bool m_full_test_scen_full_flag = false;
static bool m_full_test_init_excute_flag = false;
static int m_full_test_scen_state = FULL_TEST_NONE_S;
static uint32_t m_full_test_scheduler_state_tick;

NRF_QUEUE_DEF(uint8_t, m_full_test_scheduler, 10 , NRF_QUEUE_MODE_NO_OVERFLOW);
APP_TIMER_DEF(m_full_test_timer_id);

static void full_test_scheduler_state_machine(void * p_context);

static void full_test_timer_init(void)
{
    uint32_t      err_code;
    err_code = app_timer_create(&m_full_test_timer_id, APP_TIMER_MODE_REPEATED, full_test_scheduler_state_machine);
    APP_ERROR_CHECK(err_code);
}

static void full_test_timer_start(void)
{
    uint32_t      err_code;
    err_code = app_timer_start(m_full_test_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

static void full_test_mode_start(full_test_scheduler_state_e mode)
{
    nrf_queue_push(&m_full_test_scheduler,(uint8_t*)&mode);
}

static void full_test_on_cBle_evt(cBle_event_e cBle_evt, void *p_param)
{
    switch (cBle_evt)
    {
        case cBle_GAP_CONNECTED:
            m_full_test_ble_connect_flag = true;
            break;
        case cBle_GAP_DISCONNECTED:
            break;
        default:
            // No implementation needed.
            break;
    }
}

static void full_test_nus_send_data(char module)
{
    static uint8_t nus_send_buffer[20];
    uint32_t      err_code=0;
    memset(nus_send_buffer,0xFF,20);
    if(ble_connect_on) {
        switch(module)
        {
            case 'T':
                nus_send_buffer[0]='T';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.temperature_data,2);
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 3);
                break;
            case 'G':
                nus_send_buffer[0]='G';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.gps_data,8);
                nus_send_buffer[9]= m_nus_service_parameter.gps_cn0[0];
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 10);
                break;
            case 'W':
                nus_send_buffer[0]='W';
                memcpy(&nus_send_buffer[1],m_nus_service_parameter.wifi_data,6);
                nus_send_buffer[7]= m_nus_service_parameter.wifi_rssi[0];
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 8);
                break;
            case 'B':
                nus_send_buffer[0]='B';
                nus_send_buffer[1]= m_nus_service_parameter.battery_volt[0];
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 2);
                break;
            case 'm':  // magnet enable data
                nus_send_buffer[0]='m';
                nus_send_buffer[1]= magnet_status;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 2);
                break;
            case 's':
                nus_send_buffer[0]='s';
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            case 'i':
                nus_send_buffer[0]='i';
                nus_send_buffer[1]=(uint8_t)((m_module_parameter.gps_acquire_tracking_time_sec >> 8) & 0xFF);
                nus_send_buffer[2]=(uint8_t)(m_module_parameter.gps_acquire_tracking_time_sec & 0xFF);
                nus_send_buffer[3]=2;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 4);
                break;
            case 1:  //TRANSMIT_OK = 1;     devie => phone
                nus_send_buffer[0]=1;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            case 2:  //TRANSMIT_ERR = 1;     devie => phone
                nus_send_buffer[0]=2;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            default:
                break;
        }

        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

void cfg_full_test_nus_recv_data_handler(const uint8_t * p_data, uint16_t length)
{
    if(length > 0)
    {
        switch(p_data[0])
        {
            case 'e':
                full_test_nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone    
                m_module_waitMS_N_reset_req = 1000;
                break;

            case '!':  //full test mode enter
                if(length == 2)
                {
                    switch(p_data[1])
                    {
                        case 'e':
                            full_test_nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                            nus_disconnect_reset = true;
                            break;
                        case 'f':
                            full_test_mode_start(FULL_TEST_FULL_TEST_START_S);
                            break;
                        case 'i':
                            full_test_nus_send_data('i');
                            break;
                        case 'w':
                            full_test_mode_start(FULL_TEST_WIFI_S);
                            break;
                        case 'g':
                            full_test_mode_start(FULL_TEST_GPS_S);
                            break;
                        case 's':
                            full_test_mode_start(FULL_TEST_SIGFOX_S);
                            break;
                        case 'b':
                            full_test_mode_start(FULL_TEST_BATTERY_CHECK_S);
                            break;
                        case 'x':
                            m_full_test_running_flag = true;
                            full_test_nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone   
                            break;
                        default:
                            break;
                    }
                }
                break;

            case 'R':  //target reset
                full_test_nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                nus_disconnect_reset = true;
                break;

            default:
                cPrintLog(CDBG_BLE_ERR, "Unknown cfg_full_test_nus_cmd:0x%02x\n", p_data[0]);
                break;
        }
    }
}

static void set_full_test_state(full_test_scheduler_state_e state)
{
    m_full_test_init_excute_flag = true; /*for run initial*/
    m_full_test_scheduler_state_tick = 0; /*for timeout check*/
    m_full_test_scen_state = state;
}

full_test_scheduler_state_e full_test_mode_state_next(void)
{
    full_test_scheduler_state_e next_state = FULL_TEST_IDLE_SLEEP_S;
    switch(m_full_test_scen_state)
    {
        case FULL_TEST_NONE_S:
            next_state = FULL_TEST_INIT_S;
            break;
        case FULL_TEST_INIT_S:
            next_state = FULL_TEST_IDLE_SLEEP_S;
            break;
        case FULL_TEST_FULL_TEST_START_S:
            next_state = FULL_TEST_BATTERY_CHECK_S;
            break;
        case FULL_TEST_BATTERY_CHECK_S:
            if(m_full_test_scen_full_flag)
                next_state = FULL_TEST_GPS_S;
            break;
        case FULL_TEST_GPS_S:
            if(m_full_test_scen_full_flag)
                next_state = FULL_TEST_WIFI_S;
            break;
        case FULL_TEST_WIFI_S:
            if(m_full_test_scen_full_flag)
                next_state = FULL_TEST_SIGFOX_S;
            break;
        case FULL_TEST_SIGFOX_S:
            if(m_full_test_scen_full_flag)
                next_state = FULL_TEST_IDLE_SLEEP_S;
            break;
        case FULL_TEST_IDLE_SLEEP_S:
            break;
    }
    return next_state;
}

static void full_test_scheduler_state_machine(void * p_context)
{
    uint32_t err_code;
    uint8_t test_mode;
    
    UNUSED_PARAMETER(p_context);
    m_full_test_mode_tick ++;
    m_full_test_scheduler_state_tick ++;

    switch(m_full_test_scen_state)
    {
        case FULL_TEST_NONE_S:
            set_full_test_state(full_test_mode_state_next());
            break;
        case FULL_TEST_INIT_S:
            cPrintLog(CDBG_FCTRL_INFO, "==Full test Init!\n");
            cfg_sigfox_downlink_on_off(false);
            cfg_sigfox_set_senk_testmode_enable(false);
            set_full_test_state(full_test_mode_state_next());
            break;
        case FULL_TEST_FULL_TEST_START_S:
            m_full_test_scen_full_flag = true;
            set_full_test_state(full_test_mode_state_next());
            break;
        case FULL_TEST_BATTERY_CHECK_S:
#ifdef USR_MODULE_DEF_BATTERY_ADC_INPUT
            if(m_full_test_init_excute_flag)
            {
                cfg_battery_check_start();
                m_full_test_init_excute_flag = false;
            }
            else
            {
                uint16_t avg_batt_lvl_in_milli_volts;
                if(m_full_test_scheduler_state_tick > (APP_MAIN_SCHEDULE_HZ/2))  //wait about 500ms
                {
                    avg_batt_lvl_in_milli_volts = get_avg_batt_lvl_in_milli_volts();
                    m_nus_service_parameter.battery_volt[0] = avg_batt_lvl_in_milli_volts;
                    if(!m_full_test_scen_full_flag)full_test_nus_send_data('B');
                    set_full_test_state(full_test_mode_state_next());
                }
            }
#else
            set_full_test_state(full_test_mode_state_next());
#endif
            break;
        case FULL_TEST_GPS_S:
            if(m_full_test_init_excute_flag)
            {
                module_parameter_set_val(module_parameter_item_gps_operation_mode, 1);
                if(cGps_status_available() == CGPS_Result_OK)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "GPS Start\n");
                    cGps_nmea_acquire_request();
                    m_full_test_init_excute_flag = false;
                }
                else
                {
                    if(m_full_test_scheduler_state_tick % 10 == 0)cPrintLog(CDBG_FCTRL_ERR, "GPS busy!\n");
                }
            }
            else
            {
                int get_nmea_result = cGps_acquire_tracking_check();
                char *ns, *latitude, *ew, *longitude;
                uint8_t *output;
                if(get_nmea_result == CGPS_Result_OK)
                {
                    //position fix
                    get_NMEA_Location(&latitude, &longitude);
                    cGps_nmea_get_bufPtr(&output);
//                    memcpy(m_nus_service_parameter.gps_data, output, 8);
//                    m_nus_service_parameter.gps_cn0[0] = (uint8_t)cGps_get_cn0_avg();
                    full_test_nus_send_data('G');
                    set_full_test_state(full_test_mode_state_next());
                }
                else
                {
                    if((get_nmea_result !=  CGPS_Result_Busy) && !cGps_bus_busy_check())
                    {
                        //position not fix
                        set_full_test_state(full_test_mode_state_next());
                    }
                }
            }
            break;
        case FULL_TEST_WIFI_S:
            if(m_full_test_init_excute_flag)
            {
                if(cWifi_ap_scan_req() == CWIFI_Result_OK)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "WIFI Start\n");
                    m_full_test_init_excute_flag = false;
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module! send NUll data!\n");
                    set_full_test_state(full_test_mode_state_next());
                }
            }
            else
            {
                bool wifi_go_next_state = false;
                uint32_t get_cnt = 0;
                uint8_t *bssidBuf;
                int32_t *rssi;
                int wifi_result;
                int get_bssid_result;
                if(!cWifi_is_scan_state() && !cWifi_bus_busy_check())  //wait scan
                {
                    get_bssid_result = cWifi_get_BSSIDs_bufPtr(&bssidBuf);
                    if(get_bssid_result == CWIFI_Result_OK)
                        cWifi_get_scan_result(&get_cnt, NULL, &rssi, NULL);
                    else if(get_bssid_result == CWIFI_Result_NoData)
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI NoData! send NUll data!\n");
                    else
                        cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module! send NUll data!\n");
                    memcpy(m_nus_service_parameter.wifi_data, bssidBuf, sizeof(m_nus_service_parameter.wifi_data));
                    m_nus_service_parameter.wifi_rssi[0] = (int8_t)rssi[0];
                    m_nus_service_parameter.wifi_rssi[1] = (int8_t)rssi[1];
                    m_nus_service_parameter.module = 'W';
                    full_test_nus_send_data('W');
                    wifi_go_next_state = true;
                }
                if(wifi_go_next_state)
                {
                    if(get_cnt > 0)
                    {
                        int8_t rssi_s8 = 0;
                        if(rssi)rssi_s8=(uint8_t)rssi[0];
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Scan OK! Mac:%02x%02x%02x%02x%02x%02x RSSI:%d\n", bssidBuf[0], bssidBuf[1], bssidBuf[2], bssidBuf[3], bssidBuf[4], bssidBuf[5], (int)rssi_s8);
                    }
                    else
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Null Data!\n");
                    }
                    set_full_test_state(full_test_mode_state_next());
                }
            }
            break;
        case FULL_TEST_SIGFOX_S:
            if(m_full_test_init_excute_flag)
            {
                if(sigfox_get_state() == SETUP_S)
                {
                    uint8_t send_data[12] ={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B};
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX Start\n");
                    cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
                    cfg_sigfox_timers_start();
                    m_full_test_init_excute_flag = false;
                }
                else
                {
                    if(m_full_test_scheduler_state_tick % 10 == 0)cPrintLog(CDBG_FCTRL_ERR, "SIGFOX busy!\n");
                }
            }
            else
            {
                if(sigfox_check_exit_excute())
                {
                    cfg_sigfox_timers_stop();
                    sigfox_set_state(SETUP_S);
                    set_full_test_state(full_test_mode_state_next());
                    if(m_full_test_running_flag)
                    {
                        full_test_nus_send_data('T');
                        full_test_nus_send_data('B');
                        full_test_nus_send_data('m');
                    }
                    else
                    {
                        full_test_nus_send_data('s');
                    }
                }
            }
            break;
        case FULL_TEST_IDLE_SLEEP_S:
            if(m_full_test_init_excute_flag)
            {
                cPrintLog(CDBG_FCTRL_INFO, "==Full test Sleep!\n");
                m_full_test_scen_full_flag = false;
                m_full_test_init_excute_flag = false;
            }
            if(nrf_queue_pop(&m_full_test_scheduler, &test_mode) == NRF_SUCCESS)
            {
                cPrintLog(CDBG_MAIN_LOG, "==Full test Wakeup! State:%d\n", test_mode);
                set_full_test_state((full_test_scheduler_state_e)test_mode);
            }
            break;
    }

    if(m_full_test_ble_connect_flag && m_full_test_running_flag)
    {
        if(m_full_test_mode_tick > (CFG_FULL_TEST_MODE_TIMEOUT_SEC_RUNNING_WAIT * APP_MAIN_SCHEDULE_HZ))
        {
            cPrintLog(CDBG_MAIN_LOG, "==Full test Timeout Reset!\n");
            cfg_board_reset();
        }
    }
    else
    {
        if(m_full_test_mode_tick > (CFG_FULL_TEST_MODE_TIMEOUT_SEC_CONNECT_WAIT * APP_MAIN_SCHEDULE_HZ))
        {
            cPrintLog(CDBG_MAIN_LOG, "==Full test Timeout Reset![Not Con]\n");
            cfg_board_reset();
        }
    }
}

__WEAK void cfg_full_test_mode_enter(void)
{
    ret_code_t err_code;
    char device_name[32];
    uint32_t dev_name_size;

    cPrintLog(CDBG_MAIN_LOG, "==Full test mode enter!!\n");
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    cfg_ble_stack_init(full_test_on_cBle_evt);
    sd_power_dcdc_mode_set(1);
    cfg_board_pwr_mgmt_init();

    cfg_nvm_init();
    cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);
    cfg_board_gpio_set_default_gps();

    cfg_i2c_master_init();
#ifndef CDEV_DISABLE_ACC_MODULE_LIB
    cfg_bma250_early_init();
#ifndef CDEV_ACC_MODULE
    cfg_bma250_early_init();
    cfg_bma250_req_suppend_mode();
#endif
#endif

    cfg_board_init();
#ifdef CDEV_GPS_MODULE  //gps chip config init
    cGps_power_control(false, false);  //gps power off
#endif
#ifdef USR_MODULE_DEF_BATTERY_ADC_INPUT
    cfg_battery_check_init(USR_MODULE_DEF_BATTERY_ADC_INPUT); //for adc with battery check
#endif
    dev_name_size = sprintf(device_name, "BleTestMode");
    cfg_ble_gap_params_init((const uint8_t *)device_name, dev_name_size);  //advertising_info_init
    cfg_ble_peer_manager_init(false);
    cfg_ble_gatt_init();
    cfg_ble_services_init(false, true, cfg_full_test_nus_recv_data_handler);
    cfg_ble_advertising_init(MSEC_TO_UNITS(1000, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, NULL);
    cfg_ble_conn_params_init();
    cfg_ble_advertising_start();

    full_test_timer_init();
    full_test_timer_start();

    while(1)
    {
        cfg_scen_main_handler();
        cfg_board_power_manage();
    }
}
#endif  //FEATURE_CFG_FULL_TEST_CMD_WITH_BLE_NUS
