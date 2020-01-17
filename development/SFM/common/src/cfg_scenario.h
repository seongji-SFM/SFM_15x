/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
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
 * @brief handle module scenario.
 *
 * This file contains the module scenario.
 */


#ifndef __CFG_SCENARIO_H__
#define __CFG_SCENARIO_H__

#include <stdbool.h>
#include "cfg_config_defines.h"

typedef enum
{
    main_wakeup_reason_normal = 0,
    main_wakeup_reason_powerup = 1,
    main_wakeup_reason_powerdown = 2,
    main_wakeup_reason_key_event = 3,
    main_wakeup_reason_magnetic_event_att = 4,
    main_wakeup_reason_magnetic_event_det = 5,
    main_wakeup_reason_acc_event_shock = 6,
    main_wakeup_reason_acc_event_no_motion = 7,
    main_wakeup_reason_acc_event_motion = 8,
    main_wakeup_reason_acc_event_slope = 9,
    main_wakeup_reason_tmp_sensor_event_high =10,
    main_wakeup_reason_tmp_sensor_event_low = 11,
    main_wakeup_reason_ambient_light_sensor_event_high = 12,
    main_wakeup_reason_ambient_light_sensor_event_low = 13,
    main_wakeup_reason_type_max
}main_wakeup_reason_type; 

typedef enum
{
    NONE,
    ACC,
    MAIN_SCENARIO_LOOP,
    TMP,            /* TMP102 */
    BLE,            /**< Bluetooth mode, the application acts a simulated Heart Rate sensor. */
    GPS,            /**< Gazell mode, the application acts as a 'Gazell Device'. */
    WIFI,
    BLE_SCAN,
    SIGFOX,
#if defined(CDEV_SIGFOX_MONARCH_MODULE)
    SIGFOX_CHECK_RUN_SCAN_RC,
    SIGFOX_SCAN_RC,
#endif
    IDLE,
    BATTERY_CHECK,
    ALS,             /* NOA1305 Ambient Light Sensor */
    WAIT,

    //This should be located last.
    MAX_DO_NOTHING
}module_mode_t;

typedef enum
{
    GPS_START,    
    GPS_WORK,
    GPS_END
}gps_module_work_e;

#ifdef __cplusplus
extern "C" {
#endif

extern bool test_nus_full_tracking_mode;
extern bool nus_disconnect_reset;
extern int m_module_waitMS_N_powerdown_req;
extern bool nus_disconnect_powerdown;
extern int main_wakeup_reason;
extern bool main_wkup_key_detected;
extern bool main_button_detected;
extern bool main_magnet_detected;
extern bool main_EXTSEN_ISR_detected;
extern uint32_t main_Sec_tick;
extern module_mode_t m_module_mode;

void cfg_scen_wakeup_request(main_wakeup_reason_type reason);
void cfg_scen_powerdown_request(int delayMS, bool send_poweroff_msg);
bool cfg_scen_check_sleep_state(void);
void cfg_scen_init(void);
void cfg_scen_main_handler(void);



#ifdef __cplusplus
}
#endif
#endif //__CFG_SCENARIO_H__
