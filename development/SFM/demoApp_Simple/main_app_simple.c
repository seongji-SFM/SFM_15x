#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_bas.h"
#include "nrf_sdm.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "peer_manager.h"
#include "nrf_drv_gpiote.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "hardfault.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_mbr.h"
#include "nrf_queue.h"

#include "cfg_config.h"
#include "cfg_scenario.h"
#include "cfg_sigfox_module.h"
#include "cfg_gps_module.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_ble_ctrl.h"
#include "cfg_special_boot_mode.h"
#include "cfg_user_cmd_proc.h"
#include "cfg_adc_battery_check.h"
#include "cfg_nRF52_peripherals.h"
#include "cfg_nus_cmd_proc.h"
#include "main_app_simple.h"
#include "main_peripherals.h"

typedef enum
{
    MAIN_NONE_S,
    MAIN_INIT_S,
    MAIN_WAKEUP_S,
    MAIN_GPS_S,
    MAIN_WIFI_S,
    MAIN_SIGFOX_CHECK_RUN_SCAN_RC_S,
    MAIN_SIGFOX_SCAN_RC,
    MAIN_SIGFOX_S,
    MAIN_IDLE_SLEEP_S,
    MAIN_STATE_MAX_S
}main_scheduler_state_e;

typedef enum
{
    WAKEUP_NORMAL               = 0,
    WAKEUP_POWER_ON             = 1,
    WAKEUP_KEY_WKUP             = 2,
    WAKEUP_BLE_WAKEUP_EVENT     = 3,
    WAKEUP_TIMER                = 4,
    WAKEUP_REASON_MAX
}main_scheduler_wakeup_reason_e;

typedef struct
{
    //wifi mac
    uint8_t     wifi_get_mac_flag;
    uint8_t     wifi_get_mac_cnt;
    uint8_t     wifi_dummy[2];  //for align4
    uint8_t     wifi_mac[4/*CWIFI_BSSID_CNT*/][CWIFI_BSSID_SIZE];
    int32_t     wifi_rssi[4/*CWIFI_BSSID_CNT*/];

    //gps data
    uint8_t     gps_data_flag;
    uint8_t     gps_dummy[3];  //for align4
    uint32_t    gps_CN0_avg;
    char        gps_ns[4];
    char        gps_ew[4];
    char        gps_latitude[20];
    char        gps_longitude[20];
    uint8_t     gps_output[8];
}main_collected_data_t;

#define IACC_DATA_STORE_CNT 10

typedef struct
{
    //gpio pin status
    uint8_t     gpio_pin_lvl_WKUP;
    uint8_t     gpio_pin_dummy[3];  //for align4

    //internal accel data
    uint16_t    iacc_last_data_flag;
    uint16_t    iacc_sensitivity;
    int16_t     iacc_last_data_x;
    int16_t     iacc_last_data_y;
    int16_t     iacc_last_data_z;
    uint8_t     iacc_dump_data_flag;
    uint8_t     iacc_dump_data_idx;
    int16_t     iacc_dump_data_x[IACC_DATA_STORE_CNT];
    int16_t     iacc_dump_data_y[IACC_DATA_STORE_CNT];
    int16_t     iacc_dump_data_z[IACC_DATA_STORE_CNT];
}main_peripherals_data_t;


APP_TIMER_DEF(m_board_peripherals_timer_id);
APP_TIMER_DEF(m_main_timer_id);
NRF_QUEUE_DEF(uint8_t, m_main_wakeup_event_q, 10 , NRF_QUEUE_MODE_NO_OVERFLOW);

main_collected_data_t           m_main_cur_data;
main_collected_data_t           m_main_last_data;
main_peripherals_data_t         m_main_peripherals_data;

main_scheduler_state_e m_main_scheduler_state;
bool m_main_scheduler_init_excute_flag;
bool m_main_scheduler_wakeup_flag = false;
int /*main_scheduler_wakeup_reason_e*/ m_main_scheduler_wakeup_reason;
int m_main_scheduler_state_tick = 0;
uint32_t m_main_scheduler_wakeup_timer_sec = MAIN_APP_WAKEUP_TIMER_INTERVAL_SEC;

bool m_board_early_init_peripherals_flag = false;
bool m_board_init_peripherals_flag = false;

void board_early_init_peripherals(void);
void board_init_peripherals(void);
void main_scheduler_start(void);
void main_scheduler_wakeup_major(main_scheduler_wakeup_reason_e reason);
void main_scheduler_wakeup_minor(main_scheduler_wakeup_reason_e reason);

void board_peripherals_handler(void * p_context)
{
    int16_t accX = 0, accY = 0, accZ = 0;
    uint8_t     gpio_pin_lvl_WKUP;


    internal_Acc_Get_XYZ(&accX, &accY, &accZ);
    m_main_peripherals_data.iacc_last_data_x = accX;
    m_main_peripherals_data.iacc_last_data_y = accY;
    m_main_peripherals_data.iacc_last_data_z = accZ;
    if(!m_main_peripherals_data.iacc_last_data_flag)
    {
        m_main_peripherals_data.iacc_sensitivity = internal_Acc_Get_Sensitivity();
        m_main_peripherals_data.iacc_last_data_flag = 1;
    }
    if(++m_main_peripherals_data.iacc_dump_data_idx >= IACC_DATA_STORE_CNT)
    {
        m_main_peripherals_data.iacc_dump_data_idx = 0;
    }
    m_main_peripherals_data.iacc_dump_data_x[m_main_peripherals_data.iacc_dump_data_idx] = accX;
    m_main_peripherals_data.iacc_dump_data_y[m_main_peripherals_data.iacc_dump_data_idx] = accY;
    m_main_peripherals_data.iacc_dump_data_z[m_main_peripherals_data.iacc_dump_data_idx] = accZ;
    if((m_main_peripherals_data.iacc_dump_data_flag == 0) && (m_main_peripherals_data.iacc_dump_data_idx == 0))
    {
        m_main_peripherals_data.iacc_dump_data_flag = 1;
    }

#if 0  //print ACCEL Val 1 sec for test
    {
        static unsigned int cnt;
        if(cnt++ % 10 == 0)
        {
            cPrintLog(CDBG_GSEN_INFO, "IAcc[X:%d,Y:%d,Z:%d]\n", m_main_peripherals_data.iacc_last_data_x, m_main_peripherals_data.iacc_last_data_y, m_main_peripherals_data.iacc_last_data_z);
        }
    }
#endif
    gpio_pin_lvl_WKUP = nrf_gpio_pin_read(PIN_DEF_WKUP);
    if(gpio_pin_lvl_WKUP != m_main_peripherals_data.gpio_pin_lvl_WKUP)
    {
        cPrintLog(CDBG_EXT_SEN_INFO, "WKUP:%d to %d\n", m_main_peripherals_data.gpio_pin_lvl_WKUP, gpio_pin_lvl_WKUP);
        if(m_main_peripherals_data.gpio_pin_lvl_WKUP && !gpio_pin_lvl_WKUP)  //wakeup key released
        {
            main_scheduler_wakeup_minor(WAKEUP_KEY_WKUP);
//            main_scheduler_wakeup_major(WAKEUP_KEY_WKUP);
        }
        m_main_peripherals_data.gpio_pin_lvl_WKUP = gpio_pin_lvl_WKUP;
    }
}

void board_peripherals_timer_create(void)
{
    app_timer_timeout_handler_t timeout_handler;
    volatile uint32_t      err_code;

    err_code = app_timer_create(&m_board_peripherals_timer_id, APP_TIMER_MODE_REPEATED, board_peripherals_handler);
    APP_ERROR_CHECK(err_code);
}

void board_peripherals_timer_start(bool startFlag)
{
    static bool bStarted = false;

    if(startFlag)
    {
        if(!bStarted)
        {
            cPrintLog(CDBG_FCTRL_INFO, "board_peripherals_timer start\n");
            memset(&m_main_peripherals_data, 0, sizeof(m_main_peripherals_data));
            app_timer_start(m_board_peripherals_timer_id, APP_TIMER_TICKS(100), NULL);
            bStarted = true;
        }
    }
    else
    {
        if(bStarted)
        {
            cPrintLog(CDBG_FCTRL_INFO, "board_peripherals_timer stop\n");
            app_timer_stop(m_board_peripherals_timer_id);
            bStarted = false;
        }
    }
}

void board_Init_ModuleGPIO_Config(void)
{
#if 0
    nrf_gpio_cfg_input(PIN_DEF_INT1_ACC, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_AIN0, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_AIN1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_WKUP, NRF_GPIO_PIN_PULLDOWN);  //high active key (about 4k ohm pullup, pull-down resistance is 11~16 Kohm (typ. 13))
    nrf_gpio_cfg_input(PIN_DEF_STATE0, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_I2CS_I2C0_SCL_DBG, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_I2CS_I2C0_SDA_DBG, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_NFC1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_DEF_NFC2, NRF_GPIO_PIN_NOPULL);
#else
    nrf_gpio_cfg_input(PIN_DEF_WKUP, NRF_GPIO_PIN_PULLDOWN);  //high active key (about 4k ohm pullup, pull-down resistance is 11~16 Kohm (typ. 13))
#endif
    m_main_peripherals_data.gpio_pin_lvl_WKUP = nrf_gpio_pin_read(PIN_DEF_WKUP);
}

void board_deepsleep_peripherals(void)
{
    if(!m_board_early_init_peripherals_flag)
        board_early_init_peripherals();
    if(!m_board_init_peripherals_flag)
        board_init_peripherals();
    board_peripherals_timer_start(false);
    internal_Acc_Shutdown();                //i2c0
#if 0
    nrf_gpio_cfg_default(PIN_DEF_INT1_ACC);
    nrf_gpio_cfg_default(PIN_DEF_AIN0);
    nrf_gpio_cfg_default(PIN_DEF_AIN1);
    nrf_gpio_cfg_default(PIN_DEF_WKUP);
    nrf_gpio_cfg_default(PIN_DEF_STATE0);
    nrf_gpio_cfg_default(PIN_DEF_I2CS_I2C0_SCL_DBG);
    nrf_gpio_cfg_default(PIN_DEF_I2CS_I2C0_SDA_DBG);
    nrf_gpio_cfg_default(PIN_DEF_NFC1);
    nrf_gpio_cfg_default(PIN_DEF_NFC2);
#else
    nrf_gpio_cfg_default(PIN_DEF_WKUP);
#endif
}

void board_early_init_peripherals(void)
{
    if(!m_board_early_init_peripherals_flag)
    {
        m_board_early_init_peripherals_flag = true;
    }
}

void board_init_peripherals(void)
{
    if(!m_board_init_peripherals_flag)
    {
        board_Init_ModuleGPIO_Config();
        internal_Acc_CheckInit();           //i2c0
        m_board_init_peripherals_flag = true;
    }
}

void main_make_sigfox_send_data(void)
{
    uint8_t send_data[SIGFOX_SEND_PAYLOAD_SIZE] = {0,};

    cfg_bin_2_hexadecimal(send_data, sizeof(send_data), (char *)frame_data);
}

void cfg_scen_wakeup_request(main_wakeup_reason_type reason)  //it is called by nus_recv_data_handler()
{
    cPrintLog(CDBG_FCTRL_INFO, "wakeup req in : %d\n", reason);
    main_scheduler_wakeup_minor(WAKEUP_BLE_WAKEUP_EVENT);
}

void main_scheduler_set_state(main_scheduler_state_e new_state)
{
    m_main_scheduler_state = new_state;
    m_main_scheduler_init_excute_flag = true;
    m_main_scheduler_state_tick = 0;
}
main_scheduler_state_e main_get_next_main_state(bool operation_success_flag)
{
    main_scheduler_state_e next_state;
    switch(m_main_scheduler_state)
    {
        case MAIN_NONE_S:
            next_state = MAIN_INIT_S;
            break;
        case MAIN_INIT_S:
            next_state = MAIN_WAKEUP_S;
            break;
        case MAIN_WAKEUP_S:
            next_state = MAIN_GPS_S;
            break;
        case MAIN_GPS_S:
            if(operation_success_flag)
                next_state = MAIN_WIFI_S;
            else
                next_state = MAIN_WIFI_S;
            break;
        case MAIN_WIFI_S:
#ifdef CDEV_SIGFOX_MONARCH_MODULE
            if(operation_success_flag)
                next_state = MAIN_SIGFOX_CHECK_RUN_SCAN_RC_S;
            else
                next_state = MAIN_SIGFOX_CHECK_RUN_SCAN_RC_S;
#else
            if(operation_success_flag)
                next_state = MAIN_SIGFOX_S;
            else
                next_state = MAIN_SIGFOX_S;
#endif
            break;
        case MAIN_SIGFOX_CHECK_RUN_SCAN_RC_S:
            if(operation_success_flag)
                next_state = MAIN_SIGFOX_SCAN_RC;
            else
                next_state = MAIN_SIGFOX_S;
            break;
        case MAIN_SIGFOX_SCAN_RC:
            if(operation_success_flag)
                next_state = MAIN_SIGFOX_S;
            else
                next_state = MAIN_IDLE_SLEEP_S;
            break;
        case MAIN_SIGFOX_S:
            next_state = MAIN_IDLE_SLEEP_S;
            break;

        case MAIN_IDLE_SLEEP_S:
            next_state = MAIN_WAKEUP_S;
            break;

        default:
            cPrintLog(CDBG_FCTRL_ERR, "next state not defined! %d\n", m_main_scheduler_state);
            next_state = MAIN_IDLE_SLEEP_S;
            break;
    }
    return next_state;
}

void main_scheduler_state_machine(void * p_context)
{
    m_main_scheduler_state_tick++;
    switch(m_main_scheduler_state)
    {
        case MAIN_NONE_S:  //first state
            main_scheduler_set_state(main_get_next_main_state(true));
            break;

        case MAIN_INIT_S:
            m_main_scheduler_wakeup_reason = WAKEUP_POWER_ON;
            board_peripherals_timer_create();
            board_peripherals_timer_start(true);
            main_scheduler_set_state(main_get_next_main_state(true));
            break;

        case MAIN_WAKEUP_S:
            cPrintLog(CDBG_FCTRL_ERR, "Wakeup! sec tick:%d, reason:%d\n", main_Sec_tick, m_main_scheduler_wakeup_reason);
            main_scheduler_set_state(main_get_next_main_state(true));
            break;

        case MAIN_GPS_S:
            if(m_main_scheduler_init_excute_flag)
            {
                if(cGps_status_available() == CGPS_Result_OK)
                {
                    m_module_parameter.gps_acquire_tracking_time_sec = MAIN_APP_GPS_ACQUIRE_TRACKING_TIME_SEC;
                    m_module_parameter.gps_operation_mode = MAIN_APP_GPS_OPERATION_MODE;  //0 : smart mode, 1 : manual mode
                    cPrintLog(CDBG_FCTRL_INFO, "GPS Start! time[sec]:%d, op mode:%d(0:smart, 1:manual), hdop:%d\n", m_module_parameter.gps_acquire_tracking_time_sec, m_module_parameter.gps_operation_mode, get_hdop_value_check());
                    cGps_nmea_acquire_request();
                    m_main_scheduler_init_excute_flag = false;
                }
                else
                {
                    if(m_main_scheduler_state_tick % 10 == 0)cPrintLog(CDBG_FCTRL_ERR, "GPS busy!\n");
                }
            }
            else
            {
                int get_nmea_result = cGps_acquire_tracking_check();
                unsigned int gps_CN0_avg;
                char *ns, *latitude, *ew, *longitude;
                uint8_t *output;

                if(get_nmea_result == CGPS_Result_OK)
                {
                    //position fix
                    if(get_NMEA_Location(&latitude, &longitude) == CGPS_Result_OK)
                    {
                        get_NMEA_Direction(&ns, &ew);
                        cGps_nmea_get_bufPtr(&output);
                        gps_CN0_avg = cGps_get_cn0_avg();
                        m_main_cur_data.gps_CN0_avg = gps_CN0_avg;
                        cPrintLog(CDBG_FCTRL_INFO, "GPS fix! sec[%d], CN0 Avg:%d\n", (int)(m_main_scheduler_state_tick/APP_MAIN_SCHEDULE_HZ), (int)gps_CN0_avg);
                        cPrintLog(CDBG_FCTRL_INFO, "GPS Location Data : %s, %s, %s, %s\n", ns, latitude, ew, longitude);
                        cPrintLog(CDBG_FCTRL_INFO, "GPS Output Data : 0x%02x%02x%02x%02x%02x%02x%02x%02x\n", output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7]);
                        strcpy(m_main_cur_data.gps_ns, ns);
                        strcpy(m_main_cur_data.gps_latitude, latitude);
                        strcpy(m_main_cur_data.gps_ew, ew);
                        strcpy(m_main_cur_data.gps_longitude, longitude);
                        memcpy(m_main_cur_data.gps_output, output, 8);
                        m_main_cur_data.gps_data_flag = true;
                        main_scheduler_set_state(main_get_next_main_state(true));
                    }
                }
                else
                {
                    if((get_nmea_result !=  CGPS_Result_Busy) && !cGps_bus_busy_check())
                    {
                        //position not fix
                        cPrintLog(CDBG_FCTRL_INFO, "GPS not fix! sec[%d]\n", (int)(m_main_scheduler_state_tick/APP_MAIN_SCHEDULE_HZ));
                        main_scheduler_set_state(main_get_next_main_state(false));
                    }
                }
            }
            break;

        case MAIN_WIFI_S:
            if(m_main_scheduler_init_excute_flag)
            {
                int wifi_result;
                cWifi_Set_Scan_Type(MAIN_APP_WIFI_SCAN_TYPE);
                wifi_result = cWifi_ap_scan_req();  //scan request (normal scan)
                if(wifi_result == CWIFI_Result_OK)
                {
                    cTBC_write_state_noti("WifiScan");
                    cPrintLog(CDBG_FCTRL_INFO, "WIFI Start\n");
                    m_main_scheduler_init_excute_flag = false;
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_INFO, "Not Availalble Wifi Module!\n");
                    main_scheduler_set_state(main_get_next_main_state(false));
                }
            }
            else
            {
                if(!cWifi_is_scan_state() && !cWifi_bus_busy_check())  //wait scan
                {
                    uint32_t getIdx;
                    uint32_t getCnt = 0;
                    uint8_t *getSsid, *getBssid;
                    int32_t *getRssi;                   
                    cWifi_get_scan_result(&getCnt, &getSsid, &getRssi, &getBssid);
                    m_main_cur_data.wifi_get_mac_flag = true;
                    if(getCnt > 0)
                    {
                        m_main_cur_data.wifi_get_mac_cnt = getCnt;
                        for(getIdx = 0; getIdx < getCnt; getIdx++)
                        {
                            if(getIdx < (sizeof(m_main_cur_data.wifi_mac)/sizeof(m_main_cur_data.wifi_mac[0])))
                            {
                                memcpy(m_main_cur_data.wifi_mac[getIdx], &getBssid[CWIFI_BSSID_SIZE*getIdx], sizeof(m_main_cur_data.wifi_mac[0]));
                                m_main_cur_data.wifi_rssi[getIdx] = getRssi[getIdx];
                            }
                        }
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Scan OK! Cnt : %d\n", getCnt);
                        main_scheduler_set_state(main_get_next_main_state(true));
                    }
                    else
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "WIFI Scan Cnt is 0\n");
                        main_scheduler_set_state(main_get_next_main_state(false));
                    }
                }
            }
            break;
#ifdef CDEV_SIGFOX_MONARCH_MODULE
        case MAIN_SIGFOX_CHECK_RUN_SCAN_RC_S:
            if(m_module_parameter.sigfox_scan_rc_mode != RC_SCAN_DISABLE)
            {
                cPrintLog(CDBG_FCTRL_INFO, "RC SCAN mode : %d\n", m_module_parameter.sigfox_scan_rc_mode);
                main_scheduler_set_state(main_get_next_main_state(true));
            }
            else
            {
                cPrintLog(CDBG_FCTRL_INFO, "RC SCAN disabled! Sigfox Send Req! [Zone:%d]\n", m_module_parameter.sigfox_RC_number);
                main_scheduler_set_state(main_get_next_main_state(false));
            }
            break;
        case MAIN_SIGFOX_SCAN_RC:
            if(m_main_scheduler_init_excute_flag)
            {
                if(sigfox_get_state() == SETUP_S)
                {
                    scan_rc_parameter = true;
                    sigfox_rc_checked  = false;
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan Start!\n");
                    cfg_sigfox_timers_start();
                    m_main_scheduler_init_excute_flag = false;
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_ERR, "SIGFOX Scan busy!\n");
                }
            }
            else
            {
                if(sigfox_check_exit_excute())
                {
                    scan_rc_parameter = false;
                    cfg_sigfox_timers_stop();
                    sigfox_set_state(SETUP_S);
                    if(sigfox_rc_checked)  //rc scaned
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan OK! [Zone:%d] Time:%d\n", m_module_parameter.sigfox_RC_number, (int)(m_main_scheduler_state_tick/APP_MAIN_SCHEDULE_HZ));
                        main_scheduler_set_state(main_get_next_main_state(true));
                    }
                    else
                    {
                        cPrintLog(CDBG_FCTRL_INFO, "SIGFOX_RC_Scan Failed! Time:%d\n", (int)(m_main_scheduler_state_tick/APP_MAIN_SCHEDULE_HZ));
                        if(m_module_parameter.sigfox_scan_rc_mode == RC_SCAN_ALWAYS)
                        {
                            main_scheduler_set_state(main_get_next_main_state(true));
                        }
                        else if(m_module_parameter.sigfox_scan_rc_mode == RC_SCAN_ONLY_SEND_WHEN_CONFIRMED)
                        {
                            main_scheduler_set_state(main_get_next_main_state(false));
                        }
                        else
                        {
                            main_scheduler_set_state(main_get_next_main_state(false));
                        }
                    }
                }
            }
            break;
#endif
        case MAIN_SIGFOX_S:
            if(m_main_scheduler_init_excute_flag)
            {
                if(sigfox_get_state() == SETUP_S)
                {
                    main_make_sigfox_send_data();
                    cPrintLog(CDBG_FCTRL_INFO, "SIGFOX Start RC:%d, DL:%d, PUB:%d\n", m_module_parameter.sigfox_RC_number, m_module_parameter.sigfox_recv_en, m_module_parameter.sigfox_snek_testmode_enable);
                    cfg_sigfox_timers_start();
                    m_main_scheduler_init_excute_flag = false;
                }
                else
                {
                    cPrintLog(CDBG_FCTRL_ERR, "SIGFOX busy!\n");
                }
            }
            else
            {
                if(sigfox_check_exit_excute())
                {
                    cfg_sigfox_timers_stop();
                    sigfox_set_state(SETUP_S);
                    main_scheduler_set_state(MAIN_IDLE_SLEEP_S);
                }
            }
            break;

        case MAIN_IDLE_SLEEP_S:
            if(m_main_scheduler_init_excute_flag)
            {
                cPrintLog(CDBG_FCTRL_INFO, "Enter Sleep! Tick:%d\n", main_Sec_tick);
                m_main_scheduler_wakeup_reason = WAKEUP_NORMAL;
                m_main_scheduler_wakeup_flag = false;
                m_main_scheduler_init_excute_flag = false;
                memcpy(&m_main_last_data, &m_main_cur_data, sizeof(m_main_last_data));
                memset(&m_main_cur_data, 0, sizeof(m_main_cur_data));
            }
            else
            {
                uint8_t wakeup_reason;
                //wait for wakeup
                if(m_main_scheduler_wakeup_flag)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "wakeup req minor! [%d]\n", main_Sec_tick);
                    m_main_scheduler_wakeup_flag = false;
                    main_scheduler_set_state(MAIN_WAKEUP_S);
                }
                if(nrf_queue_pop(&m_main_wakeup_event_q, &wakeup_reason) == NRF_SUCCESS)
                {
                    cPrintLog(CDBG_FCTRL_INFO, "wakeup req major! [%d]\n", main_Sec_tick);
                    m_main_scheduler_wakeup_reason = wakeup_reason;
                    main_scheduler_set_state(MAIN_WAKEUP_S);
                }
                if((m_main_scheduler_wakeup_timer_sec > 0) && (m_main_scheduler_wakeup_timer_sec < (uint32_t)(m_main_scheduler_state_tick/APP_MAIN_SCHEDULE_HZ)))
                {
                    cPrintLog(CDBG_FCTRL_INFO, "wakeup req timer! [%d]\n", main_Sec_tick);
                    m_main_scheduler_wakeup_reason = WAKEUP_TIMER;
                    main_scheduler_set_state(MAIN_WAKEUP_S);
                }
            }
            break;

        default:
            cPrintLog(CDBG_FCTRL_ERR, "illegal state %d goto MAIN_IDLE_SLEEP_S\n", m_main_scheduler_state);
            main_scheduler_set_state(MAIN_IDLE_SLEEP_S);
            break;
    }
}

void main_scheduler_init(void)
{
    volatile uint32_t      err_code;

    err_code = app_timer_create(&m_main_timer_id, APP_TIMER_MODE_REPEATED, main_scheduler_state_machine);
    APP_ERROR_CHECK(err_code);
}

void main_scheduler_start(void)
{
    uint32_t      err_code;
    err_code = app_timer_start(m_main_timer_id, APP_TIMER_TICKS(APP_MAIN_SCHEDULE_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

void main_scheduler_wakeup_major(main_scheduler_wakeup_reason_e reason)  //it use queue
{
    nrf_queue_push(&m_main_wakeup_event_q,(uint8_t*)&reason);
}

void main_scheduler_wakeup_minor(main_scheduler_wakeup_reason_e reason)  //it can be ignored
{
    if(m_main_scheduler_state == MAIN_IDLE_SLEEP_S 
        && !m_main_scheduler_init_excute_flag
        && !m_main_scheduler_wakeup_flag
    )
    {
        m_main_scheduler_wakeup_reason = reason;
        m_main_scheduler_wakeup_flag = true;
    }
    else
    {
        cPrintLog(CDBG_FCTRL_INFO, "wakeup event ignored!\n");
    }
}

static void tbc_over_rtt_sec_tick_proc(void)
{
    main_Sec_tick++;
}

static void main_bypass_enter_CB(void)
{
    cPrintLog(CDBG_FCTRL_INFO, "call %s\n", __func__);
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
}

void main_basic_resource_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    cfg_ble_stack_init(NULL);
    sd_power_dcdc_mode_set(1);

    cfg_board_pwr_mgmt_init();
}

static void advertising_info_init(void)
{
    char device_name[32];
    uint32_t dev_name_size;
    sprintf(device_name, "%s_%02X%02X", CDEV_MODEL_NAME, m_module_peripheral_ID.ble_MAC[0], m_module_peripheral_ID.ble_MAC[1]);
    dev_name_size = strlen(device_name);
    cfg_ble_gap_params_init((const uint8_t *)device_name, dev_name_size);
}

static void on_cBle_adv_evt(cBle_adv_event_e cBle_adv_evt)
{
    switch (cBle_adv_evt)
    {
        case cBle_ADV_START:
            cPrintLog(CDBG_FCTRL_INFO, "Adv Started! interval:%d, time:%d\n", m_ble_adv_interval, m_ble_adv_timeout);
            break;

        case cBle_ADV_STOP:
            cPrintLog(CDBG_FCTRL_INFO, "Adv Stopped\n");
            break;
    
        default:
            break;
    }
}

void main_ble_init(void)
{
    advertising_info_init();
    cfg_ble_peer_manager_init(m_module_parameter_rebuiled_flag);
    cfg_ble_gatt_init();
    cfg_ble_services_init(m_module_parameter.fota_enable, true, nus_recv_data_handler);
    cfg_ble_advertising_init(MSEC_TO_UNITS(MAIN_APP_BLE_BEACON_INTAERVAL_MS, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, on_cBle_adv_evt);
    cfg_ble_conn_params_init();
}


void main_test_for_peripherals_current_consumption(void)
{
    //Insert the code that you want to measure the current consumption.
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

int main(void)
{
    ret_code_t err_code;
    cPrintLog(CDBG_MAIN_LOG, "\n====== %s Module Started Ver:%s bdtype:%d======\n", m_cfg_model_name, m_cfg_sw_ver, m_cfg_board_type);
    cPrintLog(CDBG_MAIN_LOG, "build date:%s, %s\n", m_cfg_build_date, m_cfg_build_time);

    //start of early init ////////
    module_parameter_early_read(); //just read only for module parameter
    m_cfg_board_shutdown_i2c_dev_func = board_deepsleep_peripherals;
    m_cfg_board_shutdown_io_reconf_func = board_Init_ModuleGPIO_Config;
    cfg_board_early_init(cfg_board_prepare_power_down);
    board_Init_ModuleGPIO_Config();

//test_for_low_current_consumption
//    cfg_board_testmode_BLE_Advertising_LowPwr(true, false, main_test_for_peripherals_current_consumption, NULL/*main_test_for_sleep_tick*/);

    main_basic_resource_init();
    cfg_nvm_init();
    cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);
    cTBC_init(dbg_i2c_user_cmd_proc, false);  //not use i2c debug pin  //for hitrun_test in factory
    cTBC_OVER_RTT_init(tbc_over_rtt_sec_tick_proc); //depend on cTBC_init() //FEATURE_CFG_RTT_MODULE_CONTROL  //for hitrun_test in factory
    cfg_board_gpio_set_default_gps();
    cfg_board_init();
    board_init_peripherals();
    main_ble_init();

    printout_ID_Info();

    main_scheduler_init();

    cTBC_check_N_enter_bypassmode(200, main_bypass_enter_CB, main_bypass_exit_CB);  //for hitrun_test in factory
    if(m_hitrun_test_flag)cfg_board_reset();

    cfg_ble_advertising_start_stop_req(true);
    main_scheduler_start();

    for (;; )
    {
        cfg_scen_main_handler();
        cfg_board_power_manage();
    }
}

/**
 * @}
 */
