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


#ifndef __CFG_BOARD_H__
#define __CFG_BOARD_H__
#include "cfg_config_defines.h"
#include "nrf_drv_spi.h"
#include "cfg_twis_board_control.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_opt3001_module.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MODULE_PARAMETER_MAGIC_TOP          (0xA55A0000 | CDEV_FS_VER)  //fs version
#define MODULE_PARAMETER_MAGIC_BOTTOM       (0xA55AABCD)


#define MAIN_IDLE_TIME_DEFAULT          43200 //43200//600  // 60 ~ 604800(sec, (60*60*24*7 : 1 week))
#define BEACON_INTERVAL_TIME_DEFAULT    2000 //1000  // 20 ~ 10240(ms) 
#define CGPS_ACQUIRE_TRACKING_TIME_SEC  75 //30 ~ 7200(sec)
#define SFM_BOOT_MODE_DEFAULT           0    //[0:normal, 1:wifi rf test, 2:wifi always on, 3:ble test, 4:gps test mode, 5:wifi rf test uart bridge, 6:sigfox uart over RTT, etc...]
#define OPERATION_MODE_DEFAULT          1    //0 : station mode, 1 : smart tracking mode, , 2 : full tracking mode(It is for engineer mode.)
#define GPS_ENABLE_DEFAULT              1
#define GPS_OPERATION_MODE_DEFAULT      0    //0 : smart mode, 1 : manual mode
#define WIFI_ENABLE_DEFAULT             1
#define BLE_BEACON_SCAN_ENABLE_DEFAULT  0
#define ACC_OPERATION_MODE_DEFAULT      1    //0:disable, 1: shock, 2: no_motion 3: motion, 4: slope
#define MAGNET_OPERATION_MODE_DEFAULT   3    //0:not use, 1: Attach, 2: Detach, 3: both (bit mask)
#define SIGFOX_RECV_EN_DEFAULT          0
#define KEY_POWER_OFF_LOCK_DEFAULT      0
#define FOTA_ENABLE_DEFAULT             1
#define DISABLE_BATTERY_POWER_DOWN_DEFAULT                  1
#define SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT                 0
#define NO_MOTION_DURATION               300    // set no motion state in no_motion_duration
#define WAIT_TIME_STATE                  1800//300 //3600   //sending msg in wait_time_state in motion state
#define DOWNLINK_VERSION                0
#define DOWNLINK_DAY                    7
#define OPTIONAL_MODE                   0
#ifdef CDEV_SIGFOX_MONARCH_MODULE
/*
1 : RC1
2 : RC2
3 : RC3C
4 : RC4
5 : RC5
6 : RC6
7 : RC101
*/
#define SIGFOX_RC_NUMBER_MIN                1
#define SIGFOX_RC_NUMBER_MAX                7
#define SIGFOX_RC_NUMBER_DEFAULT            1  //RC1
#if !((SIGFOX_RC_NUMBER_MIN <= SIGFOX_RC_NUMBER_DEFAULT) && (SIGFOX_RC_NUMBER_MAX >= SIGFOX_RC_NUMBER_DEFAULT))
#error "SIGFOX_RC_NUMBER_DEFAULT Error"
#endif
#endif
/* ambient light - full-scale range(lux)
0:40.95   1:81.90   2:163.80   3:327.60    4:655.20    5:1310.40 
6:2620.80 7:5241.60 8:10483.20 9:20966.40 10:41932.80 11:83865.60 */
#define AMBIENT_LIGHT_SENSOR_ENABLE_DEFAULT                 ALS_DISABLE
#define AMBIENT_LIGHT_SENSOR_INTERRUPT_MODE_DEFAULT         ALS_INTR_SET_HIGH
#define AMBIENT_LIGHT_SENSOR_INTERRUPT_VALUE_DEFAULT        OPT3001_INTR_DATA
#define TEMPERATURE_SENSOR_ENABLE_DEFAULT                   TMP_DISABLE
#define TEMPERATURE_SENSOR_INTERRUPT_MODE_DEFAULT           TMP_INTR_SET_LOWHIGH
#define TEMPERATURE_SENSOR_INTERRUPT_HIGH_VALUE_DEFAULT     TMP10x_INT_HIGH    
#define TEMPERATURE_SENSOR_INTERRUPT_LOW_VALUE_DEFAULT      TMP10x_INT_LOW

#define MAIN_BOOT_NFC_UNLOCK_DEFAULT false  //use to key power off lock
#define MAIN_FOTA_ENABLE_DEFAULT true
#define MAIN_MAGNETIC_GPIO_ENABLE_DEFAULT true  //use to power off lock
#define MAIN_SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT false

//modify define
typedef enum
{
    module_comm_pwr_pon_init,
    module_comm_pwr_common,
    module_comm_pwr_sigfox,
    module_comm_pwr_gps,
    module_comm_pwr_wifi,
    module_comm_pwr_max
}module_comm_pwr_resource_e;


/**
 * @brief setting items
 */
//Put it at the bottom when adding settings.
typedef struct
{
    uint32_t    magic_top;

    //setting value here
    uint32_t    idle_time;                                  //module_parameter_item_idle_time
    uint32_t    beacon_interval;                            //module_parameter_item_beacon_interval
    uint32_t    gps_acquire_tracking_time_sec;              //module_parameter_item_gps_tracking_time_sec
    uint32_t    log_mask;                                   //module_parameter_item_log_mask

    uint8_t     boot_mode;                                  //module_parameter_item_boot_mode
    uint8_t     operation_mode;                             //module_parameter_item_operation_mode
    uint8_t     gps_enable;                                 //module_parameter_item_gps_enable
    uint8_t     wifi_enable;                                //module_parameter_item_wifi_enable

    uint8_t     ble_beacon_scan_enable;                     //module_parameter_item_ble_beacon_scan_enable
    uint8_t     acc_operation_mode;                         //module_parameter_item_acc_operation_mode
    uint8_t     magnet_operation_mode;                      //module_parameter_item_magnet_operation_mode
    uint8_t     sigfox_recv_en;                             //module_parameter_item_sigfox_recv_en
    
    uint8_t     key_power_off_lock;                         //module_parameter_item_key_power_off_lock
    uint8_t     fota_enable;                                //module_parameter_item_fota_enable
    uint8_t     disable_battery_power_down;                 //module_parameter_item_disable_battery_power_down
    uint8_t     sigfox_snek_testmode_enable;                //module_parameter_item_sigfox_snek_testmode_enable

    uint8_t     ambient_light_sensor_enable;                //module_parameter_item_ambient_light_sensor_enable
    uint8_t     ambient_light_sensor_interrupt_mode;        //module_parameter_item_ambient_light_sensor_interrupt_mode
    uint8_t     ambient_light_sensor_interrupt_value;       //module_parameter_item_ambient_light_sensor_interrupt_value

    uint8_t     temperature_sensor_enable;                  //module_parameter_item_temperature_sensor_enable
    uint8_t     temperature_sensor_interrupt_mode;          //module_parameter_item_temperature_sensor_interrupt_mode
    int8_t      temperature_sensor_interrupt_high_value;    //module_parameter_item_temperature_sensor_high_value
    int8_t      temperature_sensor_interrupt_low_value;     //module_parameter_item_temperature_sensor_low_value
    uint8_t     gps_operation_mode;                         //module_parameter_item_gps_operation_mode   

#ifdef CDEV_SIGFOX_MONARCH_MODULE
    uint8_t     sigfox_RC_number;                           //module_parameter_item_sigfox_RC_number /*1=RC1, 2=RC2, 3=RC3c, 4=RC4, 5=RC5, 6=RC6*/
#endif

    /* the registers of accelerometer    */
    uint8_t     ctrl_mode_reg;                      //module_parameter_item_ctrl_mode_reg
    uint8_t     bw_u8;                              //module_parameter_item_bw_u8
    uint8_t     sleep_durn;                         //module_parameter_item_sleep_durn
    uint8_t     range_u8;                           //module_parameter_item_range_u8
    uint8_t     interrupt_src;                      //module_parameter_item_interrupt_src
    uint8_t     thres_u8;                           //module_parameter_item_thres_u8
    uint8_t     durn_u8;                            //module_parameter_item_durn_u8
    uint8_t     intr_x;                             //module_parameter_item_intr_x
    uint8_t     intr_y;                             //module_parameter_item_intr_y
    uint8_t     intr_z;                             //module_parameter_item_intr_z
    uint8_t     angle;                              //module_parameter_item_angle

    uint32_t    no_motion_duration;
    uint32_t    wait_time_state;

    uint8_t     downlink_version;
    uint8_t     downlink_day;
    uint8_t     optional_mode;
    uint32_t    magic_bottom;
    uint32_t    guard_area_align4;
    
    //id cache value here
    uint8_t     wifi_MAC_STA[6];
    uint8_t     sigfox_device_ID[4];
    uint8_t     sigfox_pac_code[8];
    uint8_t     board_ID;
    int32_t     guard_area_align4_2;
    uint32_t    crc32;
}module_parameter_t;

#define REGISTERED_MAC_CNT_MAX 100
#define REGISTERED_MAC_MAGIC_VAL 0x20180119

typedef struct
{
    uint32_t    magic_top;

    uint32_t    wifi_mac_cnt;
    uint8_t     wifi_r_mac[REGISTERED_MAC_CNT_MAX][6];
    uint8_t     wifi_r_position[REGISTERED_MAC_CNT_MAX][8];

    uint32_t    ble_mac_cnt;
    uint8_t     ble_r_mac[REGISTERED_MAC_CNT_MAX][6];
    uint8_t     ble_r_position[REGISTERED_MAC_CNT_MAX][8];

    uint32_t    magic_bottom;
    uint32_t    crc32;
}registered_mac_t;

#define SSIDLIST_STR_SIZE_MAX 32  /*CWIFI_SSID_SIZE*/
#define SSIDLIST_ITEM_MAX 50
#define SSIDLIST_MAGIC_VAL 0x20190823

typedef struct
{
    uint32_t    magic_top;

    uint32_t    ssid_white_cnt;
    uint8_t     ssid_white_list[SSIDLIST_ITEM_MAX][SSIDLIST_STR_SIZE_MAX];

    uint32_t    ssid_black_cnt;
    uint8_t     ssid_black_list[SSIDLIST_ITEM_MAX][SSIDLIST_STR_SIZE_MAX];

    uint32_t    magic_bottom;
    uint32_t    crc32;
}ssid_list_t;

typedef struct
{
    bool        gps_status;   // 1 fix, 0 not available 
    uint8_t     gps_data[8];  //latitude 4byte, Longtitude

    bool        ble_scan_status;   // 1 scaned, 0 not available 
    uint8_t     ble_scan_data_cnt;  // data count
    uint8_t     ble_scan_data[12];  // mac data
    int8_t      ble_scan_rssi[2];   // rssi data
    bool        ble_scan_match_gps_status;  // 1 available, 0 not available (first data)
    uint8_t     ble_scan_match_gps_data[8]; // gps data
    
    bool        wifi_status;   // 1 scaned, 0 not available 
    uint8_t     wifi_data_cnt;  // data count
    uint8_t     wifi_data[12];  // mac data
    int8_t      wifi_rssi[2];   // rssi data
    bool        wifi_match_gps_status;  // 1 available, 0 not available (first data)
    uint8_t     wifi_match_gps_data[8]; // gps data
}module_peripheral_data_t;

typedef struct
{
    uint8_t     ble_MAC[6];
    uint8_t     UUID[16];
    uint8_t     wifi_MAC_STA[6];
    uint8_t     sigfox_device_ID[4];
    uint8_t     sigfox_pac_code[8];
}module_peripheral_ID_t;

#ifdef CDEV_RTC2_DATE_TIME_CLOCK
typedef struct
{
    uint16_t year;
    uint8_t  month;  //first month is 1 : 1 to 12 (not 0 to 11)
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  dayOfWeek;
} cfg_date_time_t;
#endif

typedef enum
{
    main_send_data_none = 0,
    main_send_data_gps = 1,
    main_send_data_wifi = 2,
    main_send_data_ble = 3,
    main_send_data_sensor = 4,
    main_send_data_wifi_pos = 5,
    main_send_data_ble_pos = 6,
    main_send_data_type_max
}main_send_data_type;

typedef void (*cfg_board_shutdown_peripherals_func)(void);

extern const nrf_drv_spi_config_t m_spi_config_default;
extern module_parameter_t m_module_parameter;
extern bool m_module_parameter_update_req;
extern bool m_module_parameter_write_N_reset_flag;
extern bool m_module_parameter_save_N_reset_req;
extern int m_module_waitMS_N_reset_req;

extern registered_mac_t m_registered_mac_info;
extern ssid_list_t m_ssid_list_info;

extern const char m_cfg_sw_ver[4];
extern const char m_cfg_model_name[];
extern const char m_cfg_board_type;
extern const char m_cfg_build_date[];
extern const char m_cfg_build_time[];

extern bool m_cfg_sw_reset_detected;
extern bool m_cfg_debug_interface_wake_up_detected;
extern bool m_cfg_available_bootloader_detected;
extern bool m_cfg_NFC_wake_up_detected;
extern bool m_cfg_GPIO_wake_up_detected;
extern bool m_cfg_i2c_master_init_flag;

extern module_peripheral_data_t m_module_tracking_data;
extern module_peripheral_ID_t m_module_peripheral_ID;
extern cfg_board_shutdown_peripherals_func m_cfg_shutdown_peripherals_func;

#define SIGFOX_MSG_SEND_REASON(enum_val) (enum_val)

//util function//////////////////////////////////////////////////////////////////////////
/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input binary data
 * @param[in]   input binary data size
 * @param[out]  hexadecimal data
 *
 * @return      void
 */
void cfg_bin_2_hexadecimal(const uint8_t *pBin, int binSize, char *pHexadecimal);

/**
 * @brief       Function for hexadecimal (big endian) to integer
 *
 * @param[in]   input hexadecimal data
 * @param[in]   input hexadecimal size (max 8)
 *
 * @return      integer value
 */
uint32_t cfg_hexadecimal_2_uint_val_big(const char *pHexadecimal, int len);

/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input pHexadecimal data
 * @param[in]   input HexadeccoimalByteCnt data size (byte count)
 * @param[out]  binary data
 *
 * @return      void
 */
void cfg_hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin);

/**
 * @brief       Function for string to interger
 *
 * @param[in]   input string
 *
 * @return      interger value
 */
int cfg_atoi(const char *str);

#ifdef CDEV_RTC2_DATE_TIME_CLOCK
void RTC2_date_time_clock_init_N_start(void);  //date_time
uint32_t date_time_get_timestamp(void);  //date_time
void date_time_set_timestamp(uint32_t timestamp);
void date_time_get_current_time(cfg_date_time_t *tm);
#endif

/**@brief Function for initialising I2C .
 *
 *                         
 */
void cfg_i2c_master_init(void);

/**@brief Function for uninitialising I2C.
 *
 *                         
 */
void cfg_i2c_master_uninit(void);
uint32_t cfg_i2c_master_send_General_Call_Reset(void);

/////////////////////////////////////////////////////////////////////////////////////////
bool main_schedule_state_is_idle(void);

/////////////////////////////////////////////////////////////////////////////////////////
void cfg_board_early_init(cfg_board_shutdown_peripherals_func shutdown_peripherals_func);
void cfg_board_init(void);
void cfg_board_common_power_control(module_comm_pwr_resource_e resource, bool bOn);
void cfg_board_power_manage(void);
void cfg_board_indicate_power_down(void);
void cfg_board_prepare_power_down(void);
void cfg_board_goto_power_down(void);
void cfg_board_pwr_mgmt_init(void);
bool cfg_get_ble_led_status(void);
bool cfg_ble_led_control(bool bOn);
bool cfg_get_ble_led_status(void);
bool cfg_get_led_2_status(void);
bool cfg_led_2_control(bool bOn);
bool cfg_ble_led_control(bool bOn);
void cfg_board_reset(void);
void cfg_board_check_reset_reason(void);
void cfg_board_gpio_set_default(void);
void cfg_board_gpio_set_default_gps(void);

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
