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
 * @brief control board device control.
 *
 * This file contains the control wifi modle.
 */


#ifndef __CFG_CONFIG_SETTING_H__
#define __CFG_CONFIG_SETTING_H__

typedef enum
{
    PTYPE_SIGFOX,
    PTYPE_GPS,
    PTYPE_WIFI,
    PTYPE_BLE_SCAN,
    PTYPE_ACC,
    PTYPE_TEMPERATURE,
    PTYPE_AMBIENT_LIGHT,
    peripheral_dev_type_MAX
}peripheral_dev_type;

/**
 * @brief Enumerator used for setting items
 */
   
typedef enum
{
    module_parameter_item_idle_time                                     =  0,   //MAIN_IDLE_TIME_DEFAULT                            60 ~ 604800(sec, (60*60*24*7 : 1 week))
    module_parameter_item_beacon_interval                               =  1,   //BEACON_INTERVAL_TIME_DEFAULT                      20 ~ 10240(ms) 
    module_parameter_item_gps_tracking_time_sec                         =  2,   //CGPS_ACQUIRE_TRACKING_TIME_SEC                    30 ~ 7200(sec)
    module_parameter_item_log_mask                                      =  3,   //ref CDBGOutMask (debug log mask)                  No value limit
    module_parameter_item_boot_mode                                     =  4,   //SFM_BOOT_MODE_DEFAULT                             No value limit
    module_parameter_item_operation_mode                                =  5,   //OPERATION_MODE_DEFAULT                            0 ~ 2  //0 : station mode, 1 : smart tracking mode, , 2 : full tracking mode(It is for engineer mode.)
    module_parameter_item_gps_enable                                    =  6,   //GPS_ENABLE_DEFAULT                                0 ~ 1
    module_parameter_item_wifi_enable                                   =  7,   //WIFI_ENABLE_DEFAULT                               0 ~ 1
    module_parameter_item_ble_beacon_scan_enable                        =  8,   //BLE_BEACON_SCAN_ENABLE_DEFAULT                    0 ~ 1
    module_parameter_item_acc_operation_mode                            =  9,   //ACC_OPERATION_MODE_DEFAULT                        0 ~ 4  //0:disable, 1: shock, 2: no_motion 3: motion, 4: slope
    module_parameter_item_magnet_operation_mode                         = 10,   //MAGNET_OPERATION_MODE_DEFAULT                     0 ~ 3  //0:not use, 1: Attach, 2: Detach, 3: both (bit mask)
    module_parameter_item_sigfox_recv_en                                = 11,   //SIGFOX_RECV_EN_DEFAULT                            0 ~ 1
    module_parameter_item_key_power_off_lock                            = 12,   //KEY_POWER_OFF_LOCK_DEFAULT                        0 ~ 1
    module_parameter_item_fota_enable                                   = 13,   //FOTA_ENABLE_DEFAULT                               0 ~ 1
    module_parameter_item_disable_battery_power_down                    = 14,   //DISABLE_BATTERY_POWER_DOWN_DEFAULT                0 ~ 1
    module_parameter_item_sigfox_snek_testmode_enable                   = 15,   //SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT               0 ~ 1
    module_parameter_item_ambient_light_sensor_enable                   = 16,   //AMBIENT_LIGHT_SENSOR_ENABLE_DEFAULT               0 ~ 1  //0:disable, 1: enable
    module_parameter_item_ambient_light_sensor_interrupt_mode           = 17,   //AMBIENT_LIGHT_SENSOR_INTERRUPT_MODE_DEFAULT       0 ~ 2  //0:disable, 1: int low 2: int high
    module_parameter_item_ambient_light_sensor_interrupt_value          = 18,   //AMBIENT_LIGHT_SENSOR_INTERRUPT_VALUE_DEFAULT      0 ~ 10 or 0xFF(not use)
    module_parameter_item_temperature_sensor_enable                     = 19,   //TEMPERATURE_SENSOR_ENABLE_DEFAULT                 0 ~ 1  //0:disable, 1: enable
    module_parameter_item_temperature_sensor_interrupt_mode             = 20,   //TEMPERATURE_SENSOR_INTERRUPT_MODE__DEFAULT        0 ~ 3  //0:disable, 1: int low 2: int high, 3: int low & high
    module_parameter_item_temperature_sensor_high_value                 = 21,   //TEMPERATURE_SENSOR_INTERRUPT_HIGH_VALUE_DEFAULT   -30 ~ 80 'C
    module_parameter_item_temperature_sensor_low_value                  = 22,   //TEMPERATURE_SENSOR_INTERRUPT_LOW_VALUE_DEFAULT    -30 ~ 80 'C
    module_parameter_item_gps_operation_mode                            = 23,   //GPS_OPERATION_MODE_DEFAULT                        0 ~ 1 : //0 : smart mode, 1 : manual mode
    module_parameter_item_sigfox_RC_number                              = 24,   //SIGFOX_RC_NUMBER_DEFAULT

    /* the registers of accelerometer    */
    module_parameter_item_ctrl_mode_reg                                 = 40,   //No value limit
    module_parameter_item_bw_u8                                         = 41,   //No value limit
    module_parameter_item_sleep_durn                                    = 42,   //No value limit
    module_parameter_item_range_u8                                      = 43,   //No value limit
    module_parameter_item_interrupt_src                                 = 44,   //No value limit
    module_parameter_item_thres_u8                                      = 45,   //No value limit
    module_parameter_item_durn_u8                                       = 46,   //No value limit
    module_parameter_item_intr_x                                        = 47,   //No value limit
    module_parameter_item_intr_y                                        = 48,   //No value limit
    module_parameter_item_intr_z                                        = 49,   //No value limit
    module_parameter_item_angle                                         = 50,   //No value limit
    module_parameter_item_no_motion_duration                            = 51,   //No value limit
    module_parameter_item_wait_time_state                               = 52,   //No value limit
    module_parameter_item_downlink_version                              = 53,
    module_parameter_item_downlink_day                                  = 54,
    module_parameter_item_optional_mode                                 = 55,
    module_parameter_item_max
}module_parameter_item_e;


#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
