/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
*
* The information contained herein is property of WISOL Cor.
* Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
*
* This heading must NOT be removed from
* the file.
*
*/

#include <stdint.h>
#include <string.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"

#include "cfg_dbg_log.h"
#include "cfg_config.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_sigfox_module.h"
#include "main_demoApp.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_ble_ctrl.h"
#include "cfg_user_cmd_proc.h"
#include "cfg_bma250_module.h"
#include "cfg_opt3001_module.h"
#include "cfg_adc_battery_check.h"
#include "cfg_encode_tx_data.h"
#include "cfg_adc_battery_check.h"
#include "cfg_nRF52_peripherals.h"

bool m_hitrun_test_flag = false;

#define USER_COMMAND_PARAM_SIZE_MAX 64
static int user_cmd_cmd;
static int user_cmd_param_size;
static uint8_t user_cmd_param_buf[USER_COMMAND_PARAM_SIZE_MAX];
static int8_t user_cmd_hitrun_ble_rssi;

static void user_cmd_hitrun_input_test(void)
{
    int int_val;
    int tick_val;
    uint8_t param_bin[16];
    bool all_test_OK;

    if(user_cmd_param_size == 2)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        int_val = param_bin[0];
    }
    else
    {
        int_val = 30;  //default val
    }

    if(!m_cfg_i2c_master_init_flag)cfg_i2c_master_init();
    if(cfg_peripheral_device_enable_status_get(PTYPE_ACC))
    {
        cfg_bma250_sw_reset();
    }
    if(cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE) || cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
    {
        cfg_i2c_master_send_General_Call_Reset();
    }

    m_hitrun_test_flag = true;
    cPrintLog(CDBG_MAIN_LOG, "input test:%d %d\n", user_cmd_param_size, int_val);
            
    main_magnet_detected = false;
    main_button_detected = false;

    m_nfc_tag_on = false;
    if(int_val > 128)  //disable NFC
    {
        m_nfc_tag_on = true;
        int_val -= 128;
    }

    all_test_OK = false;
    tick_val = 0;
    while(++tick_val < (int_val * 100))
    {
        nrf_delay_ms(10);
        if(
            main_magnet_detected
            && main_button_detected
            && m_nfc_tag_on
        )
        {
            nrf_delay_ms(10);
            all_test_OK = true;
            break;
        }
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "InputTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

extern uint32_t tmp102_set_config(void);
extern uint32_t tmp102_get_tmp_data_once(int16_t *tmp102a, int16_t *tmp102b);
extern uint32_t tmp102_config_interrupt_test(void);
extern uint32_t opt3001_interrupt_test(void);
extern void opt3001_set_read_testmode(void);

static void user_cmd_hitrun_sense_test(void)
{
    bool all_test_OK = true;
    bool test_result;
    int time_out;

    uint8_t param_bin[32];
    char noti_str_buf[32];
    struct bma_accel_data acc_data;
    memset(param_bin, 0, sizeof(param_bin));
    if(user_cmd_param_size == 4)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
    }
    else if(user_cmd_param_size == 28)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
    }

    m_hitrun_test_flag = true;
    cPrintLog(CDBG_MAIN_LOG, "sense test:%d\n", user_cmd_param_size);
    //ADC Test
#ifdef USR_MODULE_DEF_BATTERY_ADC_INPUT
    {
        cfg_battery_check_start();
        nrf_delay_ms(1000);  //wait adc result check plz BATTERY_LEVEL_AVG_CNT and BATTERY_LEVEL_MEAS_INTERVAL
        sprintf(noti_str_buf,"ADCTest%02x", batt_avg_report_volts);
        if(((param_bin[0] != 0) && (param_bin[1] != 0))
            && ((batt_avg_report_volts < param_bin[0]) || (param_bin[1] < batt_avg_report_volts))
        )
        {
            all_test_OK = false;
            test_result = false;
        }
        else
        {
            test_result = true;
        }
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
    }
#endif
    //i2c sensor test
    if(!m_cfg_i2c_master_init_flag)cfg_i2c_master_init();
    
    //sensor interrupt set
    cfg_bma250_init();
    cfg_bma250_interrupt_init();

    if(!cfg_peripheral_device_enable_status_get(PTYPE_TEMPERATURE) 
        && !cfg_peripheral_device_enable_status_get(PTYPE_AMBIENT_LIGHT))
    {
        cfg_tmp10xopt3001_interrupt_init();
    }

    //ACC Test
    memset(&acc_data, 0, sizeof(acc_data));
    if(cfg_bma250_sw_reset() == NRF_SUCCESS)
    {
        nrf_delay_ms(5);
        if(cfg_bma250_read_xyz(&acc_data) == true)
        {
            m_cfg_ACC_ISR_detected = false;
            cfg_bma250_ISR_pin_test();
            time_out = 400000;
            do
            {
                if(m_cfg_ACC_ISR_detected)break;
            }while(--time_out);
            if(m_cfg_ACC_ISR_detected)
            {
                sprintf(noti_str_buf,"ACCTest%02x%02x%02x", (uint8_t)acc_data.x, (uint8_t)acc_data.y, (uint8_t)acc_data.z);
                if(((param_bin[2] != 0) && (param_bin[3] != 0)) 
                    && (
                         ((acc_data.x < (int16_t)param_bin[2]) || ((int16_t)param_bin[3] < acc_data.x))
                         || ((acc_data.y < (int16_t)param_bin[4]) || ((int16_t)param_bin[5] < acc_data.y))
                         || ((acc_data.z < (int16_t)param_bin[6]) || ((int16_t)param_bin[7] < acc_data.z))
                    )  //read value check
                )
                {
                    all_test_OK = false;
                    test_result = false;
                }
                else
                {
                    test_result = true;
                }
                cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
            }
            else
            {
                cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "ACC_IsrErr");
                all_test_OK = false;
            }
            cfg_bma250_sw_reset();
            nrf_delay_ms(5);
        }
        else
        {
            cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "ACC_ReadErr");
            all_test_OK = false;
        }
    }
    else
    {
        cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "ACC_RstErr");
        all_test_OK = false;
    }

    //tmp sensor Test
    if(tmp102_set_config()==NRF_SUCCESS)
    {
        int16_t tmp_a, tmp_b;
        nrf_delay_ms(5);
        if(tmp102_get_tmp_data_once(&tmp_a, &tmp_b)==NRF_SUCCESS)
        {
            main_EXTSEN_ISR_detected = false;
            tmp102_config_interrupt_test();
            time_out = 400000;
            do
            {
                if(main_EXTSEN_ISR_detected)break;
            }while(--time_out);

            if(main_EXTSEN_ISR_detected)
            {
                sprintf(noti_str_buf,"TMPTest%02x%02x", (uint8_t)tmp_a, (uint8_t)tmp_b);
                if(((param_bin[8] != 0) && (param_bin[9] != 0)) 
                    && (((int8_t)tmp_a < (int8_t)(param_bin[8])) || ((int8_t)(param_bin[9]) < (int8_t)tmp_a))
                )  //read value check
                {
                    all_test_OK = false;
                    test_result = false;
                }
                else
                {
                    test_result = true;
                }
                cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
            }
            else
            {
                cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "ACC_IsrErr");
                all_test_OK = false;
            }
            tmp102_set_config();
        }
        else
        {
            cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "TMP_ReadErr");
            all_test_OK = false;
        }
    }
    else
    {
        cTBC_usrcmd_msg_noti(user_cmd_cmd, false, "TMP_CfgSetErr");
        all_test_OK = false;
    }

    //Ambient light sensor Test
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "SenseTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_hitrun_ble_scan_handler(const ble_gap_evt_adv_report_t * p_adv_report)
{
    const uint8_t *p_d;
    uint16_t d_len;

    p_d = p_adv_report->data.p_data;
    d_len = p_adv_report->data.len;
    if(d_len > 11 && p_d[5]=='S' && p_d[6]=='F' && p_d[7]=='M' && p_d[8]=='T' && p_d[9]=='E' && p_d[10]=='S' && p_d[11]=='T')  //SFMTEST0000
    {
        if(user_cmd_hitrun_ble_rssi == 0 || user_cmd_hitrun_ble_rssi < p_adv_report->rssi)
        {
            user_cmd_hitrun_ble_rssi = p_adv_report->rssi;
            cPrintLog(CDBG_MAIN_LOG, "ble test:%d\n", user_cmd_hitrun_ble_rssi);
        }
    }
}

static void user_cmd_hitrun_led_test(void)
{
    bool all_test_OK = true;
    bool test_result;
    int8_t wifi_rssi = 0, ble_rssi = 0;
    uint8_t gps_time = 0;
    uint8_t param_bin[32];
    char noti_str_buf[32];
    int i;
#ifdef CDEV_GPS_MODULE
    unsigned int gps_st_sec_tick;
#endif

    memset(param_bin, 0, sizeof(param_bin));
    if(user_cmd_param_size == 6)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        wifi_rssi = (int8_t)param_bin[0];
        ble_rssi = (int8_t)param_bin[1];
        gps_time = param_bin[2];
        if(wifi_rssi || ble_rssi || gps_time)
        {
            cPrintLog(CDBG_MAIN_LOG, "Rf test enable WIFI:%d BLE:%d GPS:%d\n", wifi_rssi, ble_rssi, gps_time);
        }
    }
    m_hitrun_test_flag = true;
    cPrintLog(CDBG_MAIN_LOG, "Led test:%d\n", user_cmd_param_size);

#ifdef CDEV_GPS_MODULE
    cGps_power_control(true, true);
    gps_st_sec_tick = main_Sec_tick;
#endif

    //led test 1
    cfg_ble_led_control(true);
    nrf_delay_ms(1000);

    //led test 2
#ifdef CDEV_WIFI_MODULE
    //disable wifi log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_ERR));

    cWifi_set_retry_time(1);
    
    if(cWifi_ap_get_available_first_BSSID("SFMTEST") != CWIFI_Result_OK)
    {
        cPrintLog(CDBG_MAIN_LOG, "WIFI scan Error!\n");
        all_test_OK = false;
    }
    if(all_test_OK)
    {       
        uint32_t wifi_get_cnt;
        int32_t *rssi;
        cfg_ble_led_control(false);
        
        while(!(!cWifi_is_scan_state() && !cWifi_bus_busy_check()));  //wait scan
        nrf_delay_ms(10);
        
        if(wifi_rssi)
        {
            if((cWifi_get_scan_result(&wifi_get_cnt, NULL, &rssi, NULL) == CWIFI_Result_OK)
                && (wifi_get_cnt > 0)
                && (rssi[0] > wifi_rssi)
            )
            {
                test_result = true;
                sprintf(noti_str_buf,"WIFITest%d", (int)rssi[0]);

            }
            else
            {
                test_result = false;
                all_test_OK = false;
                sprintf(noti_str_buf,"WIFITest");
                cPrintLog(CDBG_MAIN_LOG, "WIFI rssi Error!\n");
            }
            cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
        }
    }
#else
    cfg_ble_led_control(false);
    nrf_delay_ms(1000);
#endif

    //led test 3
    if(all_test_OK)
    {
        //disable sigfox log
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_SIGFOX_INFO));
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_SIGFOX_ERR));
        if( 1
#ifdef CDEV_SIGFOX_MODULE
            && (sigfox_bypass_req(NULL, NULL) == NRF_SUCCESS)
#endif
#ifdef CDEV_WIFI_MODULE
            && (cWifi_bypass_req(NULL, NULL) == CWIFI_Result_OK)
#endif
        )
        {
#ifdef CDEV_SIGFOX_MODULE
            nrf_delay_ms(1000);
            sigfox_bypass_write_request("AT$CB=-1,1\r\n", 12);
            nrf_delay_ms(1000);
            sigfox_bypass_write_request("AT$CB=-1,0\r\n", 12);
#endif
        }
        else
        {
            all_test_OK = false;
        }
    }

    //led test all
    if(all_test_OK)
    {
        cfg_ble_led_control(true);
#ifdef CDEV_WIFI_MODULE
        cWifiState_bypass_write_request("AT+LEDON\r\n", 10);
        nrf_delay_ms(500);
        cWifi_bus_enable(false);
#endif
#ifdef CDEV_SIGFOX_MODULE
        sigfox_bypass_write_request("AT$CB=-1,1\r\n", 12);
#endif
    }

    if(all_test_OK && ble_rssi)
    {
        user_cmd_hitrun_ble_rssi = 0;
        cfg_ble_scan_init();
        cfg_ble_scan_start(user_cmd_hitrun_ble_scan_handler);
        nrf_delay_ms(1000);
        for(i=0; i < 20; i++)
        {
            if(user_cmd_hitrun_ble_rssi)break;
            nrf_delay_ms(100);
        }
        cfg_ble_scan_stop();

        if(user_cmd_hitrun_ble_rssi && (user_cmd_hitrun_ble_rssi >= ble_rssi))
        {
            test_result = true;
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "BLE scan Error! %d, %d\n", ble_rssi, user_cmd_hitrun_ble_rssi);
            test_result = false;
            all_test_OK = false;
        }
        sprintf(noti_str_buf,"BLETest%d", user_cmd_hitrun_ble_rssi);
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
            
    }

    if(all_test_OK && gps_time)
    {
#ifdef CDEV_GPS_MODULE
        extern unsigned int cGps_get_cn0_avg(void);

        uint32_t    gps_acquire_tracking_time_sec_old;
        unsigned int cn0_Avg = 0;

        gps_acquire_tracking_time_sec_old = m_module_parameter.gps_acquire_tracking_time_sec;
        m_module_parameter.gps_acquire_tracking_time_sec = gps_time;

        //disable gps log
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_GPS_INFO));
        CDBG_mask_clear(CDBG_NUM2MASK(CDBG_GPS_ERR));

        while(cGps_status_available()!=CGPS_Result_OK)nrf_delay_ms(200);
        cGps_nmea_acquire_request();
        nrf_delay_ms(200);

        while(1)
        {
            if(cGps_acquire_tracking_check()==CGPS_Result_OK)break;
            if(!cGps_bus_busy_check())break;
            nrf_delay_ms(100);
        }
        while(cGps_bus_busy_check());
        m_module_parameter.gps_acquire_tracking_time_sec = gps_acquire_tracking_time_sec_old;
        cGps_power_control(true, false);
        test_result = (cGps_acquire_tracking_check()==CGPS_Result_OK);
        if(test_result)
        {
            cn0_Avg = cGps_get_cn0_avg();
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "GPS Error! %d, %d\n", cGps_acquire_tracking_check(), cGps_bus_busy_check());
            all_test_OK = false;
        }
        sprintf(noti_str_buf,"GPSTest:%04d,%04d", cn0_Avg, (main_Sec_tick - gps_st_sec_tick));
        cTBC_usrcmd_msg_noti(user_cmd_cmd, test_result, noti_str_buf);
#endif
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "LedTest");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_set_sigfox_power(void)
{
    bool all_test_OK = false;

    if(user_cmd_param_size == 2)
    {
#ifdef CDEV_SIGFOX_MODULE
        uint8_t param_bin[16];
        uint8_t sigfox_tx_pwr;

        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        sigfox_tx_pwr = param_bin[0];
        cPrintLog(CDBG_MAIN_LOG, "Sigfox Tx Pwr:%d\n", sigfox_tx_pwr);
        if(cfg_sigfox_set_powerlevel(sigfox_tx_pwr))
        {
            all_test_OK = true;
        }
#endif
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "SigfoxTxPwr");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_set_wifi_power(void)
{
    bool all_test_OK = false;
    uint8_t param_bin[16];

    if(user_cmd_param_size == 12)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, user_cmd_param_size, param_bin);
        cPrintLog(CDBG_MAIN_LOG, "WIFI Tx Tables:%d,%d,%d,%d,%d,%d\n", param_bin[0], param_bin[1], param_bin[2], param_bin[3], param_bin[4], param_bin[5]);
#ifdef CDEV_WIFI_MODULE
        if(cWifi_bypass_req(NULL, NULL) == CWIFI_Result_OK)
        {
            char sendAtCmd[32];
            int sendATCmdSize;
            int timeout;

            timeout = 5000;
            while(!cWifiState_is_bypass_mode())
            {
                if(--timeout==0)break;  //wait bypassmode
                nrf_delay_ms(1);
            }

            if(timeout > 0)
            {
/************/
//AT Cmd Examples (Tx Power Table)
/* AT+TXPREAD           // Default 524E4A444038                      */
/* RNJD@8                                                            */
/* OK                                                                */
/* AT+TXPWRITE="445544553322"   // 445544553322                      */
/* WRITTEN                                                           */
/* OK                                                                */
/* AT+TXPREAD                                                        */
/* DUDU3"                                                            */
/* OK                                                                */
/* AT+TXPWRITE="524E4A444038"  // Default 524E4A444038 rewrite       */
/* WRITTEN                                                           */
/* OK                                                                */
/* AT+TXPREAD                                                        */
/* RNJD@8                                                            */
/* OK                                                                */
/*************/
                sendATCmdSize = sprintf((char *)sendAtCmd, "AT+TXPWRITE=\"%02X%02X%02X%02X%02X%02X\"\r\n", param_bin[0], param_bin[1], param_bin[2], param_bin[3], param_bin[4], param_bin[5]);
                cPrintLog(CDBG_MAIN_LOG, "send to WIFI :%s", sendAtCmd);
                cWifiState_bypass_write_request(sendAtCmd, sendATCmdSize);
                all_test_OK = true;
                nrf_delay_ms(500);
            }
        }
#endif
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "WIFITxPwr");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_mac_list_ctrl(void)
{
/* cmd ex with PC tool : see development\SFM\documentation\Host_FT20_Protocal.txt
get wifi list cnt : <SC>048700
get ble list cnt : <SC>048701
get all wifi list : <SC>048702
get all ble list : <SC>048703
get wifi list by index : <SC>0887040063 (index 99, big endian)
get ble list by index : <SC>0887050063
clear wifi list (not saved) : <SC>048706
clear ble list (not saved) : <SC>048707 
add wifi list (not saved): <SC>248708620194874a4037.160602,127.038148 (eg. mac:620194874a40, pos : "37.160602,127.038148" -> it is wisol position)
add ble list (not saved) : <SC>248709620194874a4037.160602,127.038148
save mac list : <SC>04870A
erase all mac list : <SC>04870B
*/

    bool all_test_OK = false;
    bool cmd_result = false;
    uint8_t param_bin[32];
    char noti_str_buf[64];
    uint8_t cmd;
    int i;
    if(user_cmd_param_size >= 2)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, 2, param_bin);
        cmd = param_bin[0];
        switch(cmd)
        {
            case 0x00: //get wifi list cnt
                sprintf(noti_str_buf,"%04x", (uint16_t)m_registered_mac_info.wifi_mac_cnt);
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x01: //get ble list cnt
                sprintf(noti_str_buf,"%04x", (uint16_t)m_registered_mac_info.ble_mac_cnt);
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x02: //get all wifi list
                for(i=0; i < m_registered_mac_info.wifi_mac_cnt; i++)
                {
                    sprintf(noti_str_buf,"%04x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", (uint16_t)i
                        , m_registered_mac_info.wifi_r_mac[i][0], m_registered_mac_info.wifi_r_mac[i][1], m_registered_mac_info.wifi_r_mac[i][2]
                        , m_registered_mac_info.wifi_r_mac[i][3], m_registered_mac_info.wifi_r_mac[i][4], m_registered_mac_info.wifi_r_mac[i][5]                   
                        , m_registered_mac_info.wifi_r_position[i][0], m_registered_mac_info.wifi_r_position[i][1], m_registered_mac_info.wifi_r_position[i][2], m_registered_mac_info.wifi_r_position[i][3]
                        , m_registered_mac_info.wifi_r_position[i][4], m_registered_mac_info.wifi_r_position[i][5], m_registered_mac_info.wifi_r_position[i][6], m_registered_mac_info.wifi_r_position[i][7]
                        );
                    cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    nrf_delay_ms(1);
                }
                all_test_OK = true;
                break;
            case 0x03: //get all ble list
                for(i=0; i < m_registered_mac_info.ble_mac_cnt; i++)
                {
                    sprintf(noti_str_buf,"%04x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", (uint16_t)i
                        , m_registered_mac_info.ble_r_mac[i][0], m_registered_mac_info.ble_r_mac[i][1], m_registered_mac_info.ble_r_mac[i][2]
                        , m_registered_mac_info.ble_r_mac[i][3], m_registered_mac_info.ble_r_mac[i][4], m_registered_mac_info.ble_r_mac[i][5]                   
                        , m_registered_mac_info.ble_r_position[i][0], m_registered_mac_info.ble_r_position[i][1], m_registered_mac_info.ble_r_position[i][2], m_registered_mac_info.ble_r_position[i][3]
                        , m_registered_mac_info.ble_r_position[i][4], m_registered_mac_info.ble_r_position[i][5], m_registered_mac_info.ble_r_position[i][6], m_registered_mac_info.ble_r_position[i][7]
                        );
                    cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    nrf_delay_ms(1);
                }
                all_test_OK = true;
                break;
            case 0x04: //idx 4 byte(big endian) get wifi list by index
                if(user_cmd_param_size == 6)
                {
                    i = (int)cfg_hexadecimal_2_uint_val_big((const char *)&user_cmd_param_buf[2], 4);
                    if(i < REGISTERED_MAC_CNT_MAX)
                    {
                        sprintf(noti_str_buf,"%04x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", (uint16_t)i
                            , m_registered_mac_info.wifi_r_mac[i][0], m_registered_mac_info.wifi_r_mac[i][1], m_registered_mac_info.wifi_r_mac[i][2]
                            , m_registered_mac_info.wifi_r_mac[i][3], m_registered_mac_info.wifi_r_mac[i][4], m_registered_mac_info.wifi_r_mac[i][5]                   
                            , m_registered_mac_info.wifi_r_position[i][0], m_registered_mac_info.wifi_r_position[i][1], m_registered_mac_info.wifi_r_position[i][2], m_registered_mac_info.wifi_r_position[i][3]
                            , m_registered_mac_info.wifi_r_position[i][4], m_registered_mac_info.wifi_r_position[i][5], m_registered_mac_info.wifi_r_position[i][6], m_registered_mac_info.wifi_r_position[i][7]
                            );
                        cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    }
                }
                break;
            case 0x05: //idx 4 byte(big endian) get ble list by index
                if(user_cmd_param_size == 6)
                {
                    i = (int)cfg_hexadecimal_2_uint_val_big((const char *)&user_cmd_param_buf[2], 4);
                    if(i < REGISTERED_MAC_CNT_MAX)
                    {
                        sprintf(noti_str_buf,"%04x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", (uint16_t)i
                            , m_registered_mac_info.wifi_r_mac[i][0], m_registered_mac_info.wifi_r_mac[i][1], m_registered_mac_info.wifi_r_mac[i][2]
                            , m_registered_mac_info.wifi_r_mac[i][3], m_registered_mac_info.wifi_r_mac[i][4], m_registered_mac_info.wifi_r_mac[i][5]                   
                            , m_registered_mac_info.wifi_r_position[i][0], m_registered_mac_info.wifi_r_position[i][1], m_registered_mac_info.wifi_r_position[i][2], m_registered_mac_info.wifi_r_position[i][3]
                            , m_registered_mac_info.wifi_r_position[i][4], m_registered_mac_info.wifi_r_position[i][5], m_registered_mac_info.wifi_r_position[i][6], m_registered_mac_info.wifi_r_position[i][7]
                            );
                        cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    }
                }
                break;
            case 0x06: //clear wifi list (not saved)
                registered_mac_remove_all_wifi();
                sprintf(noti_str_buf,"%s","WifiRemoveAll");
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x07: //clear ble list (not saved)
                registered_mac_remove_all_ble();
                sprintf(noti_str_buf,"%s","BleRemoveAll");
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x08: //add wifi list (not saved)
            case 0x09: //add ble list (not saved)
                if(user_cmd_param_size >= 14)
                {
                    uint8_t mac_data[6];
                    uint8_t *pos_data_ptr = NULL;
                    uint8_t pos_data_buf[8];
                    int i;
                    char *pos_str_ptr;
                    char pos_str_buffer[36];
                    char latitude_buf[16];
                    char longitude_buf[16];
                    cfg_hexadecimal_2_bin((const char *)&user_cmd_param_buf[2], 12, mac_data);
                    if((user_cmd_param_size>14) && (user_cmd_param_size<(14+32)))  //pos data string eg. "37.160602,127.038148"
                    {
                        memset(pos_str_buffer, 0, sizeof(pos_str_buffer));  //for loast null
                        memcpy(pos_str_buffer, &user_cmd_param_buf[14], (user_cmd_param_size-14));
                        pos_str_ptr = (char *)pos_str_buffer;
                        cPrintLog(CDBG_MAIN_LOG, "pos str:%s\n", pos_str_ptr);
                        for(i=0; i<sizeof(latitude_buf); i++)
                        {
                            if((*pos_str_ptr >= '0' && *pos_str_ptr <= '9') || (*pos_str_ptr >= '.') || (*pos_str_ptr >= '-'))
                            {
                                latitude_buf[i] = *pos_str_ptr;
                            }
                            else if(*pos_str_ptr==',')
                            {
                                pos_str_ptr++;
                                latitude_buf[i] = '\0';
                                break;
                            }
                            else
                            {
                                i = sizeof(latitude_buf);
                            }
                            pos_str_ptr++;
                        }

                        if(i < sizeof(latitude_buf))
                        {
                            for(i=0; i<sizeof(longitude_buf); i++)
                            {
                                if((*pos_str_ptr >= '0' && *pos_str_ptr <= '9') || (*pos_str_ptr >= '.') || (*pos_str_ptr >= '-'))
                                {
                                    longitude_buf[i] = *pos_str_ptr;
                                }
                                else if(*pos_str_ptr == 0)
                                {
                                    longitude_buf[i] = '\0';
                                    break;
                                }
                                else
                                {
                                    i = sizeof(longitude_buf);
                                }
                                pos_str_ptr++;
                            }
                            if(i < sizeof(longitude_buf))
                            {
                                if(make_LatitudeLongitude_to_PosData(latitude_buf, longitude_buf, pos_data_buf))
                                {
                                    pos_data_ptr = pos_data_buf;
                                }
                                cPrintLog(CDBG_MAIN_LOG, "Latitude:%s,Longitude:%s\n", latitude_buf, longitude_buf);
                                cPrintLog(CDBG_MAIN_LOG, "Pos:%02x%02x%02x%02x%02x%02x%02x%02x\n", pos_data_buf[0],pos_data_buf[1],pos_data_buf[2],pos_data_buf[3],pos_data_buf[4],pos_data_buf[5],pos_data_buf[6],pos_data_buf[7]);

#if 0  //recheck pos data
                                if(make_PosData_to_LatitudeLongitude(pos_data_buf, latitude_buf, longitude_buf))
                                {
                                    cPrintLog(CDBG_MAIN_LOG, "Recheck Latitude:%s,Longitude:%s\n", latitude_buf, longitude_buf);
                                }
#endif

                            }
                        }
                    }
                    if(cmd == 0x08)  //add wifi list
                    {
                        cmd_result = registered_mac_add_wifi(mac_data, pos_data_ptr);
                    }
                    else  //add ble list
                    {
                        cmd_result = registered_mac_add_ble(mac_data, pos_data_ptr);
                    }
                }
                if(cmd == 0x08)  //add wifi list
                {
                    if(cmd_result)
                        sprintf(noti_str_buf,"%s%04x","WifiAdd", (m_registered_mac_info.wifi_mac_cnt-1));
                    else
                        sprintf(noti_str_buf,"%s","WifiAdd");
                }
                else  //add ble list
                {
                    if(cmd_result)
                        sprintf(noti_str_buf,"%s%04x","BleAdd", (m_registered_mac_info.ble_mac_cnt-1));
                    else
                        sprintf(noti_str_buf,"%s","BleAdd");
                }
                cTBC_usrcmd_msg_noti(user_cmd_cmd, cmd_result, noti_str_buf);
                all_test_OK = cmd_result;
                break;
            case 0x0A: //save mac list
                sprintf(noti_str_buf,"%s","Update");
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                registered_mac_update(false);
                break;
            case 0x0B: //factory reset mac list (do reset)
                registered_mac_update(true);
                all_test_OK = true;
                break;
            default:
                break;
        }
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "MacListCtrl");
    m_cTBC_dbg_mode_run_func = NULL;
}

static void user_cmd_ssid_list_ctrl(void)
{
/* cmd ex with PC tool : see development\SFM\documentation\Host_FT20_Protocal.txt
get white list cnt : <SC>048800
get white list all : <SC>048801
add white list (not saved) : <SC>088802wSJ*
del white list (not saved) : <SC>0888030000

get black list cnt : <SC>048804
get black list all : <SC>048805
add black list (not saved) : <SC>088806bSJ*
del black list (not saved) : <SC>0888070000

save ssid list : <SC>048808
erase all white list (not saved) : <SC>048809
erase all black list (not saved) : <SC>04880A
erase all SSID list (not saved) : <SC>04880B
*/
    bool all_test_OK = false;
    bool cmd_result = false;
    uint8_t param_bin[32];
    char noti_str_buf[64];
    uint8_t cmd;
    int i;
    if(user_cmd_param_size >= 2)
    {
        cfg_hexadecimal_2_bin((const char *)user_cmd_param_buf, 2, param_bin);
        cmd = param_bin[0];
        switch(cmd)
        {
            case 0x00: //get white list cnt
                sprintf(noti_str_buf,"%04x", (uint16_t)m_ssid_list_info.ssid_white_cnt);
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x01: //get white list all
                for(i=0; i < m_ssid_list_info.ssid_white_cnt; i++)
                {
                    sprintf(noti_str_buf,"%04d:%s", (uint16_t)i, m_ssid_list_info.ssid_white_list[i]);
                    cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    nrf_delay_ms(1);
                }
                all_test_OK = true;
                break;
            case 0x02: //add white list (not saved)
                if(user_cmd_param_size >= 2)
                {
                    char ssid_str[SSIDLIST_STR_SIZE_MAX];

                    memset(ssid_str, 0, sizeof(ssid_str));
                    memcpy(ssid_str, (const char *)&user_cmd_param_buf[2], (user_cmd_param_size-2));
                    all_test_OK = ssid_list_add_white_list(ssid_str);
                }
                break;
            case 0x03: //del white list
                if(user_cmd_param_size == 6)
                {
                    i = (int)cfg_hexadecimal_2_uint_val_big((const char *)&user_cmd_param_buf[2], 4);
                    all_test_OK = ssid_list_remove_white_list(i);
                }
                break;

            case 0x04: //get black list cnt
                sprintf(noti_str_buf,"%04x", (uint16_t)m_ssid_list_info.ssid_black_cnt);
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = true;
                break;
            case 0x05: //get black list all
                for(i=0; i < m_ssid_list_info.ssid_black_cnt; i++)
                {
                    sprintf(noti_str_buf,"%04d:%s", (uint16_t)i, m_ssid_list_info.ssid_black_list[i]);
                    cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                    nrf_delay_ms(1);
                }
                all_test_OK = true;
                break;
            case 0x06: //add black list (not saved)
                if(user_cmd_param_size >= 2)
                {
                    char ssid_str[SSIDLIST_STR_SIZE_MAX];

                    memset(ssid_str, 0, sizeof(ssid_str));
                    memcpy(ssid_str, (const char *)&user_cmd_param_buf[2], (user_cmd_param_size-2));
                    all_test_OK = ssid_list_add_black_list(ssid_str);
                }
                break;
            case 0x07: //del black list
                if(user_cmd_param_size == 6)
                {
                    i = (int)cfg_hexadecimal_2_uint_val_big((const char *)&user_cmd_param_buf[2], 4);
                    all_test_OK = ssid_list_remove_black_list(i);
                }
                break;
            case 0x08: //save ssid list
                sprintf(noti_str_buf,"%s","Update");
                cTBC_usrcmd_msg_noti(user_cmd_cmd, true, noti_str_buf);
                all_test_OK = ssid_list_update();
                break;
            case 0x09: //erase all white list
                ssid_list_remove_all_white_list();
                all_test_OK = true;
                break;
            case 0x0a: //erase all black list
                ssid_list_remove_all_black_list();
                all_test_OK = true;
                break;
            case 0x0b: //erase all SSID list
                ssid_list_remove_all_white_list();
                ssid_list_remove_all_black_list();
                all_test_OK = true;
                break;
            default:
                break;
        }
    }
    cTBC_usrcmd_msg_noti(user_cmd_cmd, all_test_OK, "SSIDLIST");
    m_cTBC_dbg_mode_run_func = NULL;
}

void dbg_i2c_user_cmd_proc(int cmd, int param_size, const uint8_t *param)
{
    cPrintLog(CDBG_MAIN_LOG, "user cmd : 0x%02x, param size:%d\n", cmd, param_size);
    switch(cmd /*CTBC_CMD_TYPE*/)
    {
        case 0x80:  //all led on for factory test  //CTBC_CMD_USER_START
            cPrintLog(CDBG_MAIN_LOG, "led on\n");
            cfg_ble_led_control(true);
            break;

        case 0x81:  //all led off for factory test
            cPrintLog(CDBG_MAIN_LOG, "led off\n");
            cfg_ble_led_control(false);
            break;

        case 0x82:  //input test
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_input_test;
            }
            break;

        case 0x83:  //sensor_test
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_sense_test;
            }
            break;

        case 0x84:  //led_test (and rf test)
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_hitrun_led_test;
            }
            break;

        case 0x85:  //Set Sigfox Tx Power
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_set_sigfox_power;
            }
            break;

        case 0x86:  //Set Wifi Tx Power in flash
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_set_wifi_power;
            }
            break;
        case 0x87:  //mac list control
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_mac_list_ctrl;
            }
            break;
        case 0x88:  //ssid list control
            if(!cTBC_bypass_mode_is_setting() ||  m_cTBC_dbg_mode_run_func)
            {
                cTBC_usrcmd_msg_noti(cmd, 0, "Busy");
            }
            else
            {
                user_cmd_cmd = cmd;
                if(param_size < sizeof(user_cmd_param_buf)){user_cmd_param_size = param_size;}
                else{user_cmd_param_size = sizeof(user_cmd_param_buf);}
                if(user_cmd_param_size > 0)memcpy(user_cmd_param_buf, param, user_cmd_param_size);
                m_cTBC_dbg_mode_run_func = user_cmd_ssid_list_ctrl;
            }
            break;

        default:
            break;
    }
}

