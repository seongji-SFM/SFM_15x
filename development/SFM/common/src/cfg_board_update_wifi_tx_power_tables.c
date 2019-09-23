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
#include "nrf_delay.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"

#ifdef CDEV_WIFI_MODULE
//#define WIFI_TX_POWER_TABLES_UPDATE_ENABLE
#ifdef WIFI_TX_POWER_TABLES_UPDATE_ENABLE
const uint8_t wifi_tx_power_table_CE[6] = {0x45, 0x3F, 0x4D, 0x45, 0x42, 0x42};  //default 0x3b, 0x3b, 0x3b, 0x3b, 0x38, 0x38
const uint8_t wifi_tx_power_table_FCC[6] = {0x44, 0x3E, 0x3C, 0x44, 0x3C, 0x38};  //default 0x3a, 0x3a, 0x3a, 0x3a, 0x32, 0x2e
const uint8_t wifi_tx_power_table_TELECT[6] = {0x46, 0x40, 0x3E, 0x46, 0x46, 0x46};  //default 0x3c, 0x3c, 0x3c, 0x3c, 0x3c, 0x3c
#endif

void cfg_board_check_update_wifi_tx_pwr_tables(void)
{
#ifdef WIFI_TX_POWER_TABLES_UPDATE_ENABLE
    uint8_t wifiAppVer;
    uint16_t wifiInitDataVer;
    uint8_t check_tx_power_buf[6];

    if(cWifi_get_version_info(&wifiAppVer, &wifiInitDataVer) && (wifiAppVer >= 3) && cWifi_get_tx_power_tables(check_tx_power_buf))
    {
        const uint8_t *p_pwr_tables;
        bool check_tx_power = false;
        if(wifiInitDataVer == 0x0302)  //ce 
        {
            p_pwr_tables = wifi_tx_power_table_CE;
            check_tx_power = true;
        }
        else if((wifiInitDataVer == 0x0402) || (wifiInitDataVer == 0x0602))  //fcc (R2, R4)
        {
            p_pwr_tables = wifi_tx_power_table_FCC;
            check_tx_power = true;
        }
        else if(wifiInitDataVer == 0x0502) //telect
        {
            p_pwr_tables = wifi_tx_power_table_TELECT;
            check_tx_power = true;
        }

        if(check_tx_power)
        {
            if(memcmp(check_tx_power_buf, p_pwr_tables, 6) != 0)
            {
                {
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
                            sendATCmdSize = sprintf((char *)sendAtCmd, "AT+TXPWRITE=\"%02X%02X%02X%02X%02X%02X\"\r\n", p_pwr_tables[0], p_pwr_tables[1], p_pwr_tables[2], p_pwr_tables[3], p_pwr_tables[4], p_pwr_tables[5]);
                            cWifiState_bypass_write_request(sendAtCmd, sendATCmdSize);
                            nrf_delay_ms(500);
                            cWifi_abort_req();  //power off wifi module
                            timeout = 1000;
                            while(--timeout)
                            {
                                if(!cWifi_is_bypass_state() && !cWifi_bus_busy_check())break;  //wait bypassmode
                                nrf_delay_ms(1);
                            }
                            cPrintLog(CDBG_MAIN_LOG, "Wifi Tx Pwr Updated!\n");
                        }
                    }
#endif
                }
                
            }
        }
    }
    else
    {
        cPrintLog(CDBG_MAIN_LOG, "=== Check WIFI Firmware ===\n");
    }
#endif
}
#endif

