/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
*
* The information contained herein is property of WISOL Cor.
* Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
*
* This heading must NOT be removed from
* the file.
*
*/
#ifndef __CFG_NUS_CMD_PROC_H__
#define __CFG_NUS_CMD_PROC_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t     module;
    uint8_t     gps_data[8];
    uint8_t     gps_cn0[1];    
    uint8_t     temperature_data[2];
    uint8_t     wifi_data[12];
    int8_t      wifi_rssi[2];
    uint8_t     battery_volt[1];
    uint8_t     acc_x[2];
    uint8_t     acc_y[2];
    uint8_t     acc_z[2];
    uint8_t     magnet_event;
    uint8_t     accellometer_event;
    uint8_t     ap_key[8];
    uint8_t     report_count[1];
    uint8_t     wifi_scan_time[1];
    uint8_t     gps_tracking_time[1];
    uint8_t     ctrl_mode_reg;
    uint8_t     bw_u8;
    uint8_t     sleep_durn;
    uint8_t     range_u8;
    uint8_t     interrupt_src;
    uint8_t     thres_u8;
    uint8_t     durn_u8;
    uint8_t     intr_x;
    uint8_t     intr_y;
    uint8_t     intr_z;
    uint8_t     lock_button;
}nus_service_parameter_t;

#ifdef __cplusplus
extern "C" {
#endif

extern bool m_nus_send_enable;
extern bool m_nus_service_flag;
extern nus_service_parameter_t m_nus_service_parameter;
extern const uint8_t m_nus_master_unlock_code[8];

void nus_module_parameter_get(void);
void nus_data_init(void);
void nus_send_data(char module);
void nus_recv_data_handler(const uint8_t * p_data, uint16_t length);


#ifdef __cplusplus
}
#endif
#endif

