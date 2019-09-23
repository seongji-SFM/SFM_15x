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

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_nus_cmd_proc.h"
#include "cfg_ble_ctrl.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_scenario.h"
#include "cfg_encode_tx_data.h"

extern bool m_acc_report_to_nus;

nus_service_parameter_t m_nus_service_parameter;
#ifdef FEATURE_CFG_BLE_UART_CONTROL
bool m_nus_send_enable = false;
bool m_nus_service_flag = false;

static uint32_t nus_send_id_pac_mac(uint8_t * send_buffer)
{
    uint32_t err_code;
    send_buffer[0]='S';
    memcpy(send_buffer+1,m_module_peripheral_ID.sigfox_device_ID,4);
    err_code = cfg_ble_nus_data_send(send_buffer, 5);

    memset(send_buffer,0xFF,20);
    send_buffer[0]='P';
    memcpy(send_buffer+1,m_module_peripheral_ID.sigfox_pac_code,8);
    err_code = cfg_ble_nus_data_send(send_buffer, 9);

    memset(send_buffer,0xFF,20);
    send_buffer[0]='M';
    memcpy(send_buffer+1,m_module_peripheral_ID.ble_MAC,6);
    err_code = cfg_ble_nus_data_send(send_buffer,7);

    return err_code;
}
void nus_module_parameter_get(void)
{
    m_nus_service_parameter.report_count[0]= (CFG_ONE_DAY_SEC / m_module_parameter.idle_time)&0x000000FF;
    m_nus_service_parameter.gps_tracking_time[0] = (uint8_t)m_module_parameter.gps_acquire_tracking_time_sec&0x000000FF;
    m_nus_service_parameter.interrupt_src = m_module_parameter.interrupt_src;
    m_nus_service_parameter.range_u8 = m_module_parameter.range_u8;
    m_nus_service_parameter.durn_u8 = m_module_parameter.durn_u8;
    m_nus_service_parameter.thres_u8 = m_module_parameter.thres_u8;
    m_nus_service_parameter.lock_button = m_module_parameter.key_power_off_lock;
}

void nus_data_init(void)
{
    memset(m_nus_service_parameter.gps_data,0,8);
    memset(m_nus_service_parameter.gps_cn0,0,1);
    memset(m_nus_service_parameter.wifi_data,0,12);
    memset(m_nus_service_parameter.temperature_data,0,2);
    m_nus_service_parameter.magnet_event = '0';
    m_nus_service_parameter.accellometer_event = '1';
}

void nus_send_data(char module)
{
    static uint8_t nus_send_buffer[20];
    int strLen;
    uint32_t      err_code=0;
    memset(nus_send_buffer,0xFF,20);
    if(ble_connect_on && m_nus_send_enable) {
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
            case 'g': /* gps power on */
                nus_send_buffer[0]='g';
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
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

            case 'A':
                nus_send_buffer[0]='A';
                memcpy(&nus_send_buffer[1],&m_nus_service_parameter.acc_x,2);
                memcpy(&nus_send_buffer[3],&m_nus_service_parameter.acc_y,2);
                memcpy(&nus_send_buffer[5],&m_nus_service_parameter.acc_z,2);
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 7);
                break;
            case 'I':
                err_code = nus_send_id_pac_mac(nus_send_buffer);
                break;
            case 'V':  //send version
                nus_send_buffer[0]='V';
                strLen = sprintf((char *)&nus_send_buffer[1], "%s_V%s", (const char *)m_cfg_model_name, (const char *)m_cfg_sw_ver);
                err_code = cfg_ble_nus_data_send(nus_send_buffer, (strLen+1));
                break;
            case 'D':  //send build date
                nus_send_buffer[0]='D';
                strLen = sprintf((char *)&nus_send_buffer[1], "D %s", (const char *)m_cfg_build_date);
                err_code = cfg_ble_nus_data_send(nus_send_buffer, (strLen+1));
                strLen = sprintf((char *)&nus_send_buffer[1], "T %s", (const char *)m_cfg_build_time);
                err_code = cfg_ble_nus_data_send(nus_send_buffer, (strLen+1));
                break;
            case 'O':
                nus_module_parameter_get();
                nus_send_buffer[0]='O';
                nus_send_buffer[1]=m_nus_service_parameter.report_count[0];
                nus_send_buffer[2]=m_nus_service_parameter.wifi_scan_time[0];
                nus_send_buffer[3]=m_nus_service_parameter.gps_tracking_time[0];
                nus_send_buffer[4]=m_nus_service_parameter.interrupt_src;
                nus_send_buffer[5]=m_nus_service_parameter.range_u8;
                nus_send_buffer[6]=m_nus_service_parameter.durn_u8;
                nus_send_buffer[7]=m_nus_service_parameter.thres_u8;
//                nus_send_buffer[8]=m_nus_service_parameter.lock_button;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 9);
                break;
            case 'Z':
                nus_send_buffer[0]='Z';
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            case 'E':
                nus_send_buffer[0]='E';
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            case 1:  //TRANSMIT_OK = 1;     devie => phone
                nus_send_buffer[0]=1;
                err_code = cfg_ble_nus_data_send(nus_send_buffer, 1);
                break;
            case 2:  //TRANSMIT_ERROR = 2;   devie => phone
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

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */

static uint8_t m_nus_setting_step = 0;  // 1 step payload size is 16 byte (1+16) -> total data size is 17byte (max data is 20byte)
void nus_recv_data_handler(const uint8_t * p_data, uint16_t length)
{
    if(length > 0)
    {
        cPrintLog(CDBG_BLE_INFO, "nus_data recv! len:%d, head:%02x\n", length, p_data[0]);

        switch(p_data[0])
        {
            case 'T':
                nus_send_data('I');
                cfg_scen_wakeup_request(main_wakeup_reason_normal);
                break;

            case 't':  //full test mode
                test_nus_full_tracking_mode = true;
                cfg_scen_wakeup_request(main_wakeup_reason_normal);
                break;

            case 'L':
                m_nus_service_flag = true;
                break;

            case 'E':
                m_nus_service_flag = false;
                break;

            case 'S':
                m_acc_report_to_nus = false;
                break;

            case 'A' :
#ifdef FEATURE_CFG_ACC_REPORT
                m_acc_report_to_nus = true;
#endif
                break;

            case 'V':  //read version
                m_acc_report_to_nus = false;
                nus_send_data('V');
                nus_send_data('O');
                break;

            case 'D':  //build date
                nus_send_data('D');
                break;

            case 'R':  //target reset
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                nus_disconnect_reset = true;
                break;

            case 'F':  //factory reset
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                cfg_nvm_factory_reset(false);  //factory work
                nus_disconnect_reset = true;
                break;

            case 'P': //force power down
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                cfg_scen_powerdown_request(1000, true);
                break;

            case 'p': //power down when disconnect
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                nus_disconnect_powerdown = true;
                break;

            case 1:   //TRANSMIT_SETTINGS_START = 1
                cPrintLog(CDBG_MAIN_LOG, "module_parameter setting start\n");                
                m_nus_setting_step = 0;
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                break;

            case 2:   //TRANSMIT_SETTINGS = 2
                if(m_nus_setting_step == 0)
                {
                    cPrintLog(CDBG_BLE_INFO, "module_parameter setting step 1, len%d\n", (int)length);
                    module_parameter_set_val(module_parameter_item_operation_mode, (unsigned int)p_data[1]);  //Mode station/tracking
                    module_parameter_set_val(module_parameter_item_idle_time, (unsigned int)(CFG_ONE_DAY_SEC/((unsigned int)p_data[2])));  // Report time
                    module_parameter_set_val(module_parameter_item_gps_enable, (unsigned int)p_data[3]);  //GPS enable
                    module_parameter_set_val(module_parameter_item_wifi_enable, (unsigned int)p_data[4]);  //WIFI enable
                    module_parameter_set_val(module_parameter_item_ble_beacon_scan_enable, (unsigned int)p_data[5]);  //BT enable
                    module_parameter_set_val(module_parameter_item_key_power_off_lock, (unsigned int)p_data[6]);  //Button disable
                    module_parameter_set_val(module_parameter_item_gps_operation_mode, (unsigned int)p_data[7]);  //GPS smart/manual
                    module_parameter_set_val(module_parameter_item_gps_tracking_time_sec, (unsigned int)p_data[8]);  //GPS time
                    module_parameter_set_val(module_parameter_item_acc_operation_mode, (unsigned int)p_data[9]);  //G sensor enable
                    module_parameter_set_val(module_parameter_item_interrupt_src, (unsigned int)p_data[10]);  //G sensor int
                    module_parameter_set_val(module_parameter_item_range_u8, (unsigned int)p_data[11]);  //G sensor range
                    module_parameter_set_val(module_parameter_item_durn_u8, (unsigned int)p_data[12]);  //G sensor duration
                    module_parameter_set_val(module_parameter_item_thres_u8, (unsigned int)p_data[13]);  //G sensor threshold
                    module_parameter_set_val(module_parameter_item_angle, (unsigned int)p_data[14]);  //G sensor angle
                    module_parameter_set_val(module_parameter_item_temperature_sensor_enable, (unsigned int)p_data[15]);  //Temp enable
                    module_parameter_set_val(module_parameter_item_temperature_sensor_interrupt_mode, (unsigned int)p_data[16]);  //Temp int
                    nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                }
                else if(m_nus_setting_step == 1)
                {
                    cPrintLog(CDBG_BLE_INFO, "module_parameter setting step 2, len%d\n", (int)length);
                    module_parameter_set_val(module_parameter_item_temperature_sensor_high_value, (unsigned int)p_data[1]);  //Temp high
                    module_parameter_set_val(module_parameter_item_temperature_sensor_low_value, (unsigned int)p_data[2]);  //Temp low
                    module_parameter_set_val(module_parameter_item_ambient_light_sensor_enable, (unsigned int)p_data[3]);  //Light enable
                    module_parameter_set_val(module_parameter_item_ambient_light_sensor_interrupt_mode, (unsigned int)p_data[4]);  //Light int
                    module_parameter_set_val(module_parameter_item_ambient_light_sensor_interrupt_value, (unsigned int)p_data[5]);  //Light threshold
                    nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                }
                else
                {
                    cPrintLog(CDBG_BLE_INFO, "module_parameter setting step error, len:%d\n", (int)length);
                    nus_send_data(2);  //TRANSMIT_ERROR = 2;   devie => phone
                }
                m_nus_setting_step++;
                break;

            case 3:   //TRANSMIT_SETTINGS_END = 3
                cPrintLog(CDBG_BLE_INFO, "module_parameter setting end\n");
                m_nus_setting_step = 0;
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                module_parameter_update();
                nus_disconnect_reset = true;  //reset after disconnect
                break;

            case 4:   //TRANSMIT_BT_START = 4
                cPrintLog(CDBG_BLE_INFO, "BLE Mac Llist Start\n");
                registered_mac_remove_all_ble();
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                break;

            case 5:   //TRANSMIT_BT_DATA = 5
                {
                    char latitude_buf[16];
                    char longitude_buf[16];
                    cPrintLog(CDBG_MAIN_LOG, "%d add BLE mac list:[%02x%02x%02x%02x%02x%02x]\n", m_registered_mac_info.ble_mac_cnt, p_data[9],p_data[10],p_data[11],p_data[12],p_data[13],p_data[14]);
                    if(make_PosData_to_LatitudeLongitude(&p_data[1], latitude_buf, longitude_buf))
                    {
                        cPrintLog(CDBG_BLE_INFO, "  Lat:%s,Lng:%s\n", latitude_buf, longitude_buf);
                    }
                    else
                    {
                        cPrintLog(CDBG_BLE_INFO, "  Pos Data decode error!\n");
                    }
                    cPrintLog(CDBG_BLE_INFO, "  Pos:%02x%02x%02x%02x%02x%02x%02x%02x\n", p_data[1],p_data[2],p_data[3],p_data[4],p_data[5],p_data[6],p_data[7],p_data[8]);
                    registered_mac_add_ble(&p_data[9], &p_data[1]);
                    nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                }
                break;

            case 6:   //TRANSMIT_BT_END = 6
                cPrintLog(CDBG_BLE_INFO, "BLE Mac Llist End. item cnt : %d\n", m_registered_mac_info.wifi_mac_cnt);
                registered_mac_update(false);
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                nus_disconnect_reset = true;  //reset after disconnect
                break;

            case 7:  //TRANSMIT_WIFI_START = 7;
                cPrintLog(CDBG_BLE_INFO, "WIFI Mac Llist Start\n");
                registered_mac_remove_all_wifi();
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                break;
            
            case 8:   //TRANSMIT_WIFI_DATA = 8;
                {
                    char latitude_buf[16];
                    char longitude_buf[16];
                    cPrintLog(CDBG_BLE_INFO, "%d add WIFI mac list:[%02x%02x%02x%02x%02x%02x]\n", m_registered_mac_info.wifi_mac_cnt, p_data[9],p_data[10],p_data[11],p_data[12],p_data[13],p_data[14]);
                    if(make_PosData_to_LatitudeLongitude(&p_data[1], latitude_buf, longitude_buf))
                    {
                        cPrintLog(CDBG_BLE_INFO, "  Lat:%s,Lng:%s\n", latitude_buf, longitude_buf);
                    }
                    else
                    {
                        cPrintLog(CDBG_BLE_INFO, "  Pos Data decode error!\n");
                    }
                    cPrintLog(CDBG_BLE_INFO, "  Pos:%02x%02x%02x%02x%02x%02x%02x%02x\n", p_data[1],p_data[2],p_data[3],p_data[4],p_data[5],p_data[6],p_data[7],p_data[8]);
                    registered_mac_add_wifi(&p_data[9], &p_data[1]);
                    nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                }
                break;
            
            case 9:   //TRANSMIT_WIFI_END = 9;
                cPrintLog(CDBG_BLE_INFO, "WIFI Mac Llist End. item cnt : %d\n", m_registered_mac_info.wifi_mac_cnt);
                registered_mac_update(false);
                nus_send_data(1);  //TRANSMIT_OK = 1;     devie => phone
                nus_disconnect_reset = true;  //reset after disconnect
                break;

            default:
                cPrintLog(CDBG_BLE_ERR, "Unknown nus header:0x%02x\n", p_data[0]);
                break;
        }
    }
}
/**@snippet [Handling the data received over BLE] */

#endif

