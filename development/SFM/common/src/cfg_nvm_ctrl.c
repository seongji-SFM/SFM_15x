/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nrf_delay.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "crc32.h"

#include "cfg_config.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_dbg_log.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_opt3001_module.h"

#define NVM_Developer_logs (0)
//#define TEST_FEATURE_MAKE_MAC_FOR_FULL_TEST

bool cfg_nvm_fs_init_flag = false;
uint32_t cfg_nvm_fs_page_size;

extern void fds_flash_bounds_set(void);
extern uint32_t fds_flash_start_addr(void);
extern uint32_t fds_flash_end_addr(void);

static bool cfg_nvm_fs_init(void);
static void cfg_nvm_fs_flash_bounds_set(void);

/*************************************************/
//support flash area
//param_fs_config : setting values, loaded to m_module_parameter
//mac_fs_config : mac table for Priority
/*************************************************/
module_parameter_t m_module_parameter;
bool m_module_parameter_update_req = false;
bool m_module_parameter_write_N_reset_flag = false;
bool m_module_parameter_rebuiled_flag = false;
static void module_parameter_fs_evt_handler(nrf_fstorage_evt_t * p_evt);
NRF_FSTORAGE_DEF(nrf_fstorage_t param_fs_config) =
{
    .evt_handler  = module_parameter_fs_evt_handler, // Function for event callbacks.
};

static void module_parameter_fs_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        cPrintLog(CDBG_FLASH_ERR, "m_param ERROR. fstorage operation.! %d, %p\n", p_evt->id, (void *)p_evt->addr);
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "m_param Write OK! %p\n", (void *)p_evt->addr);
            if(m_module_parameter_write_N_reset_flag)
            {
                cPrintLog(CDBG_FLASH_INFO, "target reset for parameter_write_N_reset!\n");
                cfg_board_reset();
            }
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "m_param erase OK! %p\n", (void *)p_evt->addr);
        } break;

        default:
            break;
    }
}

static void module_parameter_size_check(void)
{
    if(cfg_nvm_fs_page_size < sizeof(m_module_parameter))  //see FS_PAGE_SIZE_WORDS
    {
        cPrintLog(CDBG_FLASH_ERR, "m_param error fatal! page size:%d, item size:%d\n", cfg_nvm_fs_page_size, sizeof(m_module_parameter));
        nrf_delay_ms(100);
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);  //fs_config num_pages size over
    }
}

static void module_parameter_default_init(void)
{
    memset(&m_module_parameter, 0, sizeof(m_module_parameter));
    m_module_parameter.magic_top = MODULE_PARAMETER_MAGIC_TOP;

    m_module_parameter.idle_time = MAIN_IDLE_TIME_DEFAULT;
    m_module_parameter.beacon_interval = BEACON_INTERVAL_TIME_DEFAULT;
    m_module_parameter.gps_acquire_tracking_time_sec = CGPS_ACQUIRE_TRACKING_TIME_SEC;
    m_module_parameter.log_mask = CDBG_mask_val_get();
    m_module_parameter.boot_mode = SFM_BOOT_MODE_DEFAULT;
    m_module_parameter.operation_mode = OPERATION_MODE_DEFAULT;
    m_module_parameter.gps_enable = GPS_ENABLE_DEFAULT;
    m_module_parameter.wifi_enable = WIFI_ENABLE_DEFAULT;
    m_module_parameter.ble_beacon_scan_enable = BLE_BEACON_SCAN_ENABLE_DEFAULT;
    m_module_parameter.acc_operation_mode = ACC_OPERATION_MODE_DEFAULT;
    m_module_parameter.magnet_operation_mode = MAGNET_OPERATION_MODE_DEFAULT;
    m_module_parameter.sigfox_recv_en = SIGFOX_RECV_EN_DEFAULT;
    m_module_parameter.key_power_off_lock = KEY_POWER_OFF_LOCK_DEFAULT;
    m_module_parameter.fota_enable = FOTA_ENABLE_DEFAULT;
    m_module_parameter.disable_battery_power_down = DISABLE_BATTERY_POWER_DOWN_DEFAULT;
    m_module_parameter.sigfox_snek_testmode_enable = SIGFOX_SNEK_TESTMODE_ENABLE_DEFAULT;
    m_module_parameter.ambient_light_sensor_enable = AMBIENT_LIGHT_SENSOR_ENABLE_DEFAULT;
    m_module_parameter.ambient_light_sensor_interrupt_mode = AMBIENT_LIGHT_SENSOR_INTERRUPT_MODE_DEFAULT;
    m_module_parameter.ambient_light_sensor_interrupt_value = AMBIENT_LIGHT_SENSOR_INTERRUPT_VALUE_DEFAULT;
    m_module_parameter.temperature_sensor_enable = TEMPERATURE_SENSOR_ENABLE_DEFAULT;
    m_module_parameter.temperature_sensor_interrupt_mode = TEMPERATURE_SENSOR_INTERRUPT_MODE_DEFAULT;
    m_module_parameter.temperature_sensor_interrupt_high_value = TEMPERATURE_SENSOR_INTERRUPT_HIGH_VALUE_DEFAULT;
    m_module_parameter.temperature_sensor_interrupt_low_value = TEMPERATURE_SENSOR_INTERRUPT_LOW_VALUE_DEFAULT;
    m_module_parameter.gps_operation_mode = GPS_OPERATION_MODE_DEFAULT;
#ifdef CDEV_SIGFOX_MONARCH_MODULE
    m_module_parameter.sigfox_RC_number = SIGFOX_RC_NUMBER_DEFAULT;
    m_module_parameter.sigfox_scan_rc_mode = SIGFOX_SCAN_RC_MODE_DEFAULT;
#endif

    /* the registers of accelerometer    */
    m_module_parameter.ctrl_mode_reg = BMA2x2_MODE_LOWPOWER1;
    m_module_parameter.bw_u8 = BMA2x2_BW_7_81HZ;
    m_module_parameter.sleep_durn = BMA2x2_SLEEP_DURN_500MS;
    m_module_parameter.interrupt_src = 0x04;
    m_module_parameter.range_u8 = BMA2x2_RANGE_2G;
    m_module_parameter.thres_u8 = 48 ;
    m_module_parameter.durn_u8 = 2;
    m_module_parameter.intr_x = 1;
    m_module_parameter.intr_y = 1;
    m_module_parameter.intr_z = 1;

    m_module_parameter.no_motion_duration = NO_MOTION_DURATION;
    m_module_parameter.wait_time_state = WAIT_TIME_STATE;
    m_module_parameter.downlink_version = DOWNLINK_VERSION;
    m_module_parameter.downlink_day = DOWNLINK_DAY;
    m_module_parameter.optional_mode = OPTIONAL_MODE;
    m_module_parameter.magic_bottom = MODULE_PARAMETER_MAGIC_BOTTOM;
    m_module_parameter.guard_area_align4 = 0;
    
    m_module_parameter.board_ID = m_cfg_board_type;
}

static bool module_parameter_adjust_value(void)
{
    volatile int old_val;
    bool adjusted = false;
    if(((int)(m_module_parameter.idle_time) < 60) || ((int)(m_module_parameter.idle_time) > (60*60*24*7)))
    {
        adjusted = true;
        old_val = m_module_parameter.idle_time;
        if((int)(m_module_parameter.idle_time) < 60)m_module_parameter.idle_time = 60;
        else m_module_parameter.idle_time = (60*60*24*7);
        cPrintLog(CDBG_MAIN_LOG, "adjust idle_time %d to %d\n", old_val, m_module_parameter.idle_time);
    }

    if(((int)(m_module_parameter.beacon_interval) < 20) || ((int)(m_module_parameter.beacon_interval) > 10240))
    {
        adjusted = true;
        old_val = m_module_parameter.beacon_interval;
        if((int)(m_module_parameter.beacon_interval) < 20)m_module_parameter.beacon_interval = 20;
        else m_module_parameter.beacon_interval = 10240;
        cPrintLog(CDBG_MAIN_LOG, "adjust beacon_interval %d to %d\n", old_val, m_module_parameter.beacon_interval);
    }

    if(((int)(m_module_parameter.gps_acquire_tracking_time_sec) < 30) || ((int)(m_module_parameter.gps_acquire_tracking_time_sec) > 7200))
    {
        adjusted = true;
        old_val = m_module_parameter.gps_acquire_tracking_time_sec;
        if((int)(m_module_parameter.gps_acquire_tracking_time_sec) < 30)m_module_parameter.gps_acquire_tracking_time_sec = 30;
        else m_module_parameter.gps_acquire_tracking_time_sec = 7200;
        cPrintLog(CDBG_MAIN_LOG, "adjust gps_acquire_tracking_time_sec %d to %d\n", old_val, m_module_parameter.gps_acquire_tracking_time_sec);
    }

    if(((int)(m_module_parameter.gps_enable) < 0) || ((int)(m_module_parameter.gps_enable) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.gps_enable;
        if((int)(m_module_parameter.gps_enable) < 0)m_module_parameter.gps_enable = 0;
        else m_module_parameter.gps_enable = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust gps_enable %d to %d\n", old_val, m_module_parameter.gps_enable);
    }
    
    if(((int)(m_module_parameter.gps_operation_mode) < 0) || ((int)(m_module_parameter.gps_operation_mode) > 1))
    {
        adjusted = true;
        old_val = m_module_parameter.gps_operation_mode;
        if((int)(m_module_parameter.gps_operation_mode) < 0)m_module_parameter.gps_operation_mode = 0;
        else m_module_parameter.gps_operation_mode = 1;
        cPrintLog(CDBG_MAIN_LOG, "adjust gps_operation_mode %d to %d\n", old_val, m_module_parameter.gps_operation_mode);
    }

#ifdef CDEV_SIGFOX_MONARCH_MODULE
    if(((int)(m_module_parameter.sigfox_RC_number) < SIGFOX_RC_NUMBER_MIN) || ((int)(m_module_parameter.sigfox_RC_number) > SIGFOX_RC_NUMBER_MAX))
    {
        adjusted = true;
        old_val = m_module_parameter.sigfox_RC_number;
        m_module_parameter.sigfox_RC_number = SIGFOX_RC_NUMBER_DEFAULT;
        cPrintLog(CDBG_MAIN_LOG, "adjust sigfox_RC_number %d to %d\n", old_val, m_module_parameter.sigfox_RC_number);
    }
#endif

    return adjusted;
}
static uint32_t module_parameter_crc_get(void)
{
    return crc32_compute((uint8_t*)&m_module_parameter, sizeof(m_module_parameter)-4, NULL);
}

static void module_parameter_init(void)
{
    bool parameter_rebuild_req = false;

    if(!cfg_nvm_fs_init_flag)return;

    memcpy(&m_module_parameter, (void *)param_fs_config.start_addr, sizeof(m_module_parameter));

    if(!module_parameter_check_magic())
    {
        parameter_rebuild_req = true;
        cPrintLog(CDBG_FLASH_INFO, "rebuild module_parameter!\n");
    }
    else if(m_module_parameter.board_ID != m_cfg_board_type)
    {
        parameter_rebuild_req = true;
        cPrintLog(CDBG_FLASH_INFO, "board_type not same!\n");
    }
    else if(m_module_parameter.crc32 != module_parameter_crc_get())
    {
        parameter_rebuild_req = true;
        cPrintLog(CDBG_FLASH_ERR, "crc Err! module_parameter!\n");
    }

    if(!parameter_rebuild_req)
    {
        cPrintLog(CDBG_FLASH_DBG, "module_parameter loaded!\n");
    }
    else
    {
        nrf_delay_ms(1);
        cPrintLog(CDBG_FLASH_INFO, "module_parameter to default!\n");
        module_parameter_default_init();
        module_parameter_update();
        nrf_delay_ms(10);
        m_module_parameter_rebuiled_flag = true;
    }
    module_parameter_adjust_value();
    //for ID value cache
    memcpy(m_module_peripheral_ID.wifi_MAC_STA, m_module_parameter.wifi_MAC_STA, sizeof(m_module_peripheral_ID.wifi_MAC_STA));
    memcpy(m_module_peripheral_ID.sigfox_device_ID, m_module_parameter.sigfox_device_ID, sizeof(m_module_peripheral_ID.sigfox_device_ID));
    memcpy(m_module_peripheral_ID.sigfox_pac_code, m_module_parameter.sigfox_pac_code, sizeof(m_module_peripheral_ID.sigfox_pac_code));
}


void module_parameter_update(void)
{
    ret_code_t fs_ret;   
    if(cfg_nvm_fs_init_flag)
    {
        m_module_parameter.crc32 = module_parameter_crc_get();
        cPrintLog(CDBG_FLASH_INFO, "module parameter update!\n");
        fs_ret = nrf_fstorage_erase(&param_fs_config, param_fs_config.start_addr, 1, NULL);
        if (fs_ret == NRF_SUCCESS)
        {
            fs_ret = nrf_fstorage_write(&param_fs_config, param_fs_config.start_addr, &m_module_parameter, sizeof(m_module_parameter), NULL);
            if (fs_ret != NRF_SUCCESS)
            {
                cPrintLog(CDBG_FLASH_ERR, "m_param fs_store error! %d\n", fs_ret);
            }
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "m_param fs_erase error! %d\n", fs_ret);
        }
    }
}

void module_parameter_check_update(void)
{
    if(m_module_parameter_update_req)
    {
        module_parameter_update();
        m_module_parameter_update_req = false;
    }
}

unsigned int module_parameter_get_val(module_parameter_item_e item)
{
    unsigned int ret = 0;
    switch(item)
    {
        case module_parameter_item_idle_time:
            ret = m_module_parameter.idle_time;
            break;
        case module_parameter_item_beacon_interval:
            ret = m_module_parameter.beacon_interval;
            break;
        case module_parameter_item_gps_tracking_time_sec:
            ret = m_module_parameter.gps_acquire_tracking_time_sec;
            break;
        case module_parameter_item_log_mask:
            ret = m_module_parameter.log_mask;
            break;
        case module_parameter_item_boot_mode:
            ret = m_module_parameter.boot_mode;
            break;
        case module_parameter_item_operation_mode:
            ret = m_module_parameter.operation_mode;
            break;
        case module_parameter_item_gps_enable:
            ret = m_module_parameter.gps_enable;
            break;
        case module_parameter_item_wifi_enable:
            ret = m_module_parameter.wifi_enable;
            break;
        case module_parameter_item_ble_beacon_scan_enable:
            ret = m_module_parameter.ble_beacon_scan_enable;
            break;
        case module_parameter_item_acc_operation_mode:
            ret = m_module_parameter.acc_operation_mode;
            break;
        case module_parameter_item_magnet_operation_mode:
            ret = m_module_parameter.magnet_operation_mode;
            break;
        case module_parameter_item_sigfox_recv_en:
            ret = m_module_parameter.sigfox_recv_en;
            break;
        case module_parameter_item_key_power_off_lock:
            ret = m_module_parameter.key_power_off_lock;
            break;
        case module_parameter_item_fota_enable:
            ret = m_module_parameter.fota_enable;
            break;
        case module_parameter_item_disable_battery_power_down:
            ret = m_module_parameter.disable_battery_power_down;
            break;
        case module_parameter_item_sigfox_snek_testmode_enable:
            ret = m_module_parameter.sigfox_snek_testmode_enable;
            break;
        case module_parameter_item_ambient_light_sensor_enable:
            ret = m_module_parameter.ambient_light_sensor_enable;
            break;
        case module_parameter_item_ambient_light_sensor_interrupt_mode:
            ret = m_module_parameter.ambient_light_sensor_interrupt_mode;
            break;
        case module_parameter_item_ambient_light_sensor_interrupt_value:
            ret = m_module_parameter.ambient_light_sensor_interrupt_value;
            break;
        case module_parameter_item_temperature_sensor_enable:
            ret = m_module_parameter.temperature_sensor_enable;
            break;
        case module_parameter_item_temperature_sensor_interrupt_mode:
            ret = m_module_parameter.temperature_sensor_interrupt_mode;
            break;
        case module_parameter_item_temperature_sensor_high_value:
            ret = m_module_parameter.temperature_sensor_interrupt_high_value;
            break;
        case module_parameter_item_temperature_sensor_low_value:
            ret = m_module_parameter.temperature_sensor_interrupt_low_value;
            break;
        case module_parameter_item_gps_operation_mode:
            ret = m_module_parameter.gps_operation_mode;
            break;
        case module_parameter_item_sigfox_RC_number:
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)  //CDEV_SIGFOX_MONARCH_MODULE
            ret = m_module_parameter.sigfox_RC_number;
#endif
            break;
        case module_parameter_item_sigfox_scan_rc_mode:
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)  //CDEV_SIGFOX_MONARCH_MODULE
            ret = m_module_parameter.sigfox_scan_rc_mode;
#endif
            break;

        case module_parameter_item_ctrl_mode_reg:
            ret = m_module_parameter.ctrl_mode_reg;
            break;
        case module_parameter_item_bw_u8:
            ret = m_module_parameter.bw_u8;
            break;
        case module_parameter_item_sleep_durn:
            ret = m_module_parameter.sleep_durn;
            break;
        case module_parameter_item_range_u8:
            ret = m_module_parameter.range_u8;
            break;
        case module_parameter_item_interrupt_src:
            ret = m_module_parameter.interrupt_src;
            break;
        case module_parameter_item_thres_u8:
            ret = m_module_parameter.thres_u8;
            break;
        case module_parameter_item_durn_u8:
            ret = m_module_parameter.durn_u8;
            break;
        case module_parameter_item_intr_x:
            ret = m_module_parameter.intr_x;
            break;
        case module_parameter_item_intr_y:
            ret = m_module_parameter.intr_y;
            break;
        case module_parameter_item_intr_z:
            ret = m_module_parameter.intr_z;
            break;
        case module_parameter_item_angle:
            ret = m_module_parameter.angle;
            break;

        default:
            cPrintLog(CDBG_MAIN_LOG, "get_param_val Arg Error %d\n", (int)item);
            break;
    }
    return ret;
}

void module_parameter_set_val(module_parameter_item_e item, unsigned int val)
{
    switch(item)
    {
        case module_parameter_item_idle_time:
            cPrintLog(CDBG_MAIN_LOG, "wakeup time:%d to %d\n", m_module_parameter.idle_time, val);
            m_module_parameter.idle_time = (uint32_t)val;
            break;
        case module_parameter_item_beacon_interval:
            cPrintLog(CDBG_MAIN_LOG, "BLE beacon interval:%d to %d\n", m_module_parameter.beacon_interval, val);
            m_module_parameter.beacon_interval = (uint32_t)val;
            break;
        case module_parameter_item_gps_tracking_time_sec:
            cPrintLog(CDBG_MAIN_LOG, "gps acquire tracking time:%d to %d\n", m_module_parameter.gps_acquire_tracking_time_sec, val);
            m_module_parameter.gps_acquire_tracking_time_sec = (uint32_t)val;
            break;
        case module_parameter_item_log_mask:
            cPrintLog(CDBG_MAIN_LOG, "log mask:0x%08x to 0x%08x\n", m_module_parameter.log_mask, val);
            m_module_parameter.log_mask = (uint32_t)val;
            CDBG_mask_val_set(m_module_parameter.log_mask);
            break;
        case module_parameter_item_boot_mode:
            cPrintLog(CDBG_MAIN_LOG, "sfm_boot_mode:%d to %d\n", m_module_parameter.boot_mode, val);
            cPrintLog(CDBG_MAIN_LOG, "mode def:0:normal, 1:wifi rf test, 2:wifi always on, 3:ble test, 4:gps test mode\n");
            cPrintLog(CDBG_MAIN_LOG, "mode def:5:wifi rf test bridge from RTT to uart, 6:sigfox over RTT, 7:sigfox over Uart, 8:WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)\n");
            cPrintLog(CDBG_MAIN_LOG, "mode def:9:BLE Scan, A:BLE Advertising, B:GPS Bypass over NUS(GPS2NUS), C:Sigfox Bypass over NUS(SFX2NUS)\n");
            m_module_parameter.boot_mode = (uint8_t)val;
            break;
        case module_parameter_item_operation_mode:
            cPrintLog(CDBG_MAIN_LOG, "operation_mode:%d to %d\n", m_module_parameter.operation_mode, val);
            m_module_parameter.operation_mode = (uint8_t)val;
            break;
        case module_parameter_item_gps_enable:
            cPrintLog(CDBG_MAIN_LOG, "gps_enable:%d to %d\n", m_module_parameter.gps_enable, val);
            m_module_parameter.gps_enable = (uint8_t)val;
            break;
        case module_parameter_item_wifi_enable:
            cPrintLog(CDBG_MAIN_LOG, "wifi_enable:%d to %d\n", m_module_parameter.wifi_enable, val);
            m_module_parameter.wifi_enable = (uint8_t)val;
            break;
        case module_parameter_item_ble_beacon_scan_enable:
            cPrintLog(CDBG_MAIN_LOG, "ble_beacon_scan_enable:%d to %d\n", m_module_parameter.ble_beacon_scan_enable, val);
            m_module_parameter.ble_beacon_scan_enable = (uint8_t)val;
            break;
        case module_parameter_item_acc_operation_mode:
            cPrintLog(CDBG_MAIN_LOG, "acc_operation_mode:%d to %d\n", m_module_parameter.acc_operation_mode, val);
            m_module_parameter.acc_operation_mode = (uint8_t)val;
            break;
        case module_parameter_item_magnet_operation_mode:
            cPrintLog(CDBG_MAIN_LOG, "magnet_operation_mode:%d to %d\n", m_module_parameter.magnet_operation_mode, val);
            m_module_parameter.magnet_operation_mode = (uint8_t)val;
            break;
        case module_parameter_item_sigfox_recv_en:
            cPrintLog(CDBG_MAIN_LOG, "sigfox DL:%d to %d\n", m_module_parameter.sigfox_recv_en, val);
            m_module_parameter.sigfox_recv_en = (uint8_t)val;
            break;
        case module_parameter_item_key_power_off_lock:
            cPrintLog(CDBG_MAIN_LOG, "key_power_off_lock:%d to %d\n", m_module_parameter.key_power_off_lock, val);
            m_module_parameter.key_power_off_lock = (uint8_t)val;
            break;
        case module_parameter_item_fota_enable:
            cPrintLog(CDBG_MAIN_LOG, "fota_enable:%d to %d\n", m_module_parameter.fota_enable, val);
            m_module_parameter.fota_enable = (uint8_t)val;
            break;
        case module_parameter_item_disable_battery_power_down:
            cPrintLog(CDBG_MAIN_LOG, "disable_battery_power_down:%d to %d\n", m_module_parameter.disable_battery_power_down, val);
            m_module_parameter.disable_battery_power_down = (uint8_t)val;
            break;
        case module_parameter_item_sigfox_snek_testmode_enable:
            cPrintLog(CDBG_MAIN_LOG, "sigfox_snek_testmode_enable:%d to %d\n", m_module_parameter.sigfox_snek_testmode_enable, val);
            m_module_parameter.sigfox_snek_testmode_enable = (uint8_t)val;
            break;
        case module_parameter_item_ambient_light_sensor_enable:
            cPrintLog(CDBG_MAIN_LOG, "ambient_light_sensor_enable:%d to %d\n", m_module_parameter.ambient_light_sensor_enable, val);
            m_module_parameter.ambient_light_sensor_enable = (uint8_t)val;
            break;
        case module_parameter_item_ambient_light_sensor_interrupt_mode:
            cPrintLog(CDBG_MAIN_LOG, "ambient_light_sensor_interrupt_mode:%d to %d\n", m_module_parameter.ambient_light_sensor_interrupt_mode, val);
            m_module_parameter.ambient_light_sensor_interrupt_mode = (uint8_t)val;
            break;            
        case module_parameter_item_ambient_light_sensor_interrupt_value:
            cPrintLog(CDBG_MAIN_LOG, "ambient_light_sensor_interrupt_value:%d to %d\n", m_module_parameter.ambient_light_sensor_interrupt_value, val);
            m_module_parameter.ambient_light_sensor_interrupt_value = (uint8_t)val;
            break;
        case module_parameter_item_temperature_sensor_enable:
            cPrintLog(CDBG_MAIN_LOG, "temperature_sensor_enable:%d to %d\n", m_module_parameter.temperature_sensor_enable, val);
            m_module_parameter.temperature_sensor_enable = (uint8_t)val;
            break;
        case module_parameter_item_temperature_sensor_interrupt_mode:
            cPrintLog(CDBG_MAIN_LOG, "temperature_sensor_interrupt_mode:%d to %d\n", m_module_parameter.temperature_sensor_interrupt_mode, val);
            m_module_parameter.temperature_sensor_interrupt_mode = (uint8_t)val;
            break;
        case module_parameter_item_temperature_sensor_high_value:
            cPrintLog(CDBG_MAIN_LOG, "temperature_sensor_interrupt_high_value:%d to %d\n", m_module_parameter.temperature_sensor_interrupt_high_value, val);
            m_module_parameter.temperature_sensor_interrupt_high_value = (int8_t)val;
            break; 
        case module_parameter_item_temperature_sensor_low_value:
            cPrintLog(CDBG_MAIN_LOG, "temperature_sensor_interrupt_low_value:%d to %d\n", m_module_parameter.temperature_sensor_interrupt_low_value, val);
            m_module_parameter.temperature_sensor_interrupt_low_value = (int8_t)val;
            break;
        case module_parameter_item_gps_operation_mode:
            cPrintLog(CDBG_MAIN_LOG, "gps_operation_mode:%d to %d\n", m_module_parameter.gps_operation_mode, val);
            m_module_parameter.gps_operation_mode = (uint8_t)val;
            break;
        case module_parameter_item_sigfox_RC_number:
#ifdef CDEV_SIGFOX_MONARCH_MODULE
            cPrintLog(CDBG_MAIN_LOG, "sigfox_RC_number:%d to %d\n", m_module_parameter.sigfox_RC_number, val);
            m_module_parameter.sigfox_RC_number = (uint8_t)val;
#else
            cPrintLog(CDBG_MAIN_LOG, "sigfox_RC_number Not Support\n");
#endif
            break;

        case module_parameter_item_sigfox_scan_rc_mode:
#ifdef CDEV_SIGFOX_MONARCH_MODULE
            cPrintLog(CDBG_MAIN_LOG, "sigfox_scan_rc_mode:%d to %d\n", m_module_parameter.sigfox_scan_rc_mode, val);
            m_module_parameter.sigfox_scan_rc_mode = (uint8_t)val;
#else
            cPrintLog(CDBG_MAIN_LOG, "sigfox_scan_rc_mode Not Support\n");
#endif
            break;

        case module_parameter_item_ctrl_mode_reg:
            cPrintLog(CDBG_MAIN_LOG, "ACC ctrl_mode_reg:%d to %d\n", m_module_parameter.ctrl_mode_reg, val);
            m_module_parameter.ctrl_mode_reg = (uint8_t)val;
            break;
        case module_parameter_item_bw_u8:
            cPrintLog(CDBG_MAIN_LOG, "ACC bw_u8:%d to %d\n", m_module_parameter.bw_u8, val);
            m_module_parameter.bw_u8 = (uint8_t)val;
            break;
        case module_parameter_item_sleep_durn:
            cPrintLog(CDBG_MAIN_LOG, "ACC sleep_durn:%d to %d\n", m_module_parameter.sleep_durn, val);
            m_module_parameter.sleep_durn = (uint8_t)val;
            break;
        case module_parameter_item_range_u8:
            cPrintLog(CDBG_MAIN_LOG, "ACC range_u8:%d to %d\n", m_module_parameter.range_u8, val);
            m_module_parameter.range_u8 = (uint8_t)val;
            break;
        case module_parameter_item_interrupt_src:
            cPrintLog(CDBG_MAIN_LOG, "ACC interrupt_src:%d to %d\n", m_module_parameter.interrupt_src, val);
            m_module_parameter.interrupt_src = (uint8_t)val;
            break;
        case module_parameter_item_thres_u8:
            cPrintLog(CDBG_MAIN_LOG, "ACC thres_u8:%d to %d\n", m_module_parameter.thres_u8, val);
            m_module_parameter.thres_u8 = (uint8_t)val;
            break;
        case module_parameter_item_durn_u8:
            cPrintLog(CDBG_MAIN_LOG, "ACC durn_u8:%d to %d\n", m_module_parameter.durn_u8, val);
            m_module_parameter.durn_u8 = (uint8_t)val;
            break;
        case module_parameter_item_intr_x:
            cPrintLog(CDBG_MAIN_LOG, "ACC intr_x:%d to %d\n", m_module_parameter.intr_x, val);
            m_module_parameter.intr_x = (uint8_t)val;
            break;
        case module_parameter_item_intr_y:
            cPrintLog(CDBG_MAIN_LOG, "ACC intr_y:%d to %d\n", m_module_parameter.intr_y, val);
            m_module_parameter.intr_y = (uint8_t)val;
            break;
        case module_parameter_item_intr_z:
            cPrintLog(CDBG_MAIN_LOG, "ACC intr_z:%d to %d\n", m_module_parameter.intr_z, val);
            m_module_parameter.intr_z = (uint8_t)val;
            break;
        case module_parameter_item_angle:
            cPrintLog(CDBG_MAIN_LOG, "ACC angle:%d to %d\n", m_module_parameter.angle, val);
            m_module_parameter.angle = (uint8_t)val;
            break;
        case module_parameter_item_no_motion_duration:
            cPrintLog(CDBG_MAIN_LOG, "ACC no_motion_duration:%d to %d\n", m_module_parameter.no_motion_duration, val);
            m_module_parameter.no_motion_duration = (uint8_t)val;
            break;
        case module_parameter_item_wait_time_state:
            cPrintLog(CDBG_MAIN_LOG, "ACC wait_time_state:%d to %d\n", m_module_parameter.wait_time_state, val);
            m_module_parameter.wait_time_state = (uint8_t)val;
            break;
        case module_parameter_item_downlink_version:
            cPrintLog(CDBG_MAIN_LOG, "DOWNLINK VERSION:%d to %d\n", m_module_parameter.downlink_version, val);
            m_module_parameter.downlink_version = (uint8_t)val;
            break;
        case module_parameter_item_downlink_day:
            cPrintLog(CDBG_MAIN_LOG, "DOWNLINK DAY:%d to %d\n", m_module_parameter.downlink_day, val);
            m_module_parameter.downlink_day = (uint8_t)val;
            break;
        case module_parameter_item_optional_mode:
            cPrintLog(CDBG_MAIN_LOG, "OPTIONAL MODE:%d to %d\n", m_module_parameter.optional_mode, val);
            m_module_parameter.optional_mode = (uint8_t)val;
            break;

        default:
            cPrintLog(CDBG_MAIN_LOG, "set_param_val Arg Error %d, 0x%08x\n", (int)item, val);
            break;
    }
}

bool module_parameter_check_magic(void)
{
    if((m_module_parameter.magic_top == MODULE_PARAMETER_MAGIC_TOP && m_module_parameter.magic_bottom == MODULE_PARAMETER_MAGIC_BOTTOM))
    {
        return true;
    }
    return false;
}

bool module_parameter_get_bootmode(int *bootmode)
{
    if(module_parameter_check_magic())
    {
        if(bootmode)*bootmode = m_module_parameter.boot_mode;
        return true;
    }
    return false;
}

void module_parameter_early_read(void)  //just read only for module parameter
{
    cfg_nvm_fs_flash_bounds_set();
    memcpy(&m_module_parameter, (void *)param_fs_config.start_addr, sizeof(m_module_parameter));
    if(module_parameter_check_magic())  //set logmask
    {
        CDBG_mask_val_set(m_module_parameter.log_mask);
    }

}

registered_mac_t m_registered_mac_info;
static bool m_registered_mac_write_N_reset_flag = false;

static void registered_mac_fs_evt_handler(nrf_fstorage_evt_t * p_evt);
NRF_FSTORAGE_DEF(nrf_fstorage_t registered_mac_fs_config) =
{
    .evt_handler  = registered_mac_fs_evt_handler, // Function for event callbacks.
};

static void registered_mac_fs_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        cPrintLog(CDBG_FLASH_ERR, "r_mac ERROR. fstorage operation.! %d, %p\n", p_evt->id, (void *)p_evt->addr);
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "r_mac Write OK! %p\n", (void *)p_evt->addr);
            if(m_registered_mac_write_N_reset_flag)
            {
                cPrintLog(CDBG_FLASH_INFO, "target reset for parameter_write_N_reset!\n");
                cfg_board_reset();
            }
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "r_mac erase OK! %p\n", (void *)p_evt->addr);
        } break;

        default:
            break;
    }
}

static void registered_mac_size_check(void)
{
    if(cfg_nvm_fs_page_size < sizeof(m_registered_mac_info))  //see FS_PAGE_SIZE_WORDS
    {
        cPrintLog(CDBG_FLASH_ERR, "r_mac error fatal! page size:%d, item size:%d\n", cfg_nvm_fs_page_size, sizeof(m_registered_mac_info));
        nrf_delay_ms(100);
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);  //fs_config num_pages size over
    }
}

#ifdef TEST_FEATURE_MAKE_MAC_FOR_FULL_TEST
//SADH32101, SC4620R13908 mac (bootmode 8)
static const uint8_t wifi_test_mac[6] = {0x62,0x01,0x94,0x87,0x4a,0x40};
static const uint8_t wifi_ble_mac[6] = {0xC2,0x82,0x36,0x25,0x44,0x72};
   
//wisol pos
static const char *test_pos_latitude = "37.160602";
static const char *test_pos_longitude = "127.038148";
static void registered_mac_make_test_mac(void)
{
    uint8_t temp_wifi_mac[6];
    uint8_t temp_ble_mac[6];
    uint8_t temp_pos_data[8];
    int i;
    registered_mac_remove_all_wifi();
    registered_mac_remove_all_ble();

    memcpy(temp_wifi_mac, wifi_test_mac, 5);
    temp_wifi_mac[5] = 0;
    memcpy(temp_ble_mac, wifi_ble_mac, 5);
    temp_ble_mac[5] = 0;
    for(i=0; i < REGISTERED_MAC_CNT_MAX-1;i++)
    {
        registered_mac_add_wifi(temp_wifi_mac, NULL);
        registered_mac_add_ble(temp_ble_mac, NULL);
    }
    make_LatitudeLongitude_to_PosData(test_pos_latitude, test_pos_longitude, temp_pos_data);
    registered_mac_add_wifi(wifi_test_mac, temp_pos_data);
    registered_mac_add_ble(wifi_ble_mac, temp_pos_data);
}
#endif

static void registered_mac_default_init(void)
{
    m_registered_mac_info.magic_top = REGISTERED_MAC_MAGIC_VAL;
    m_registered_mac_info.magic_bottom = REGISTERED_MAC_MAGIC_VAL;
#ifdef TEST_FEATURE_MAKE_MAC_FOR_FULL_TEST
    registered_mac_make_test_mac();
#else
    registered_mac_remove_all_wifi();
    registered_mac_remove_all_ble();
#endif
}

static uint32_t registered_mac_crc_get(void)
{
    return crc32_compute((uint8_t*)&m_registered_mac_info, sizeof(m_registered_mac_info)-4, NULL);
}

static void registered_mac_init(void)
{
    bool registered_mac_rebuild_req = false;
    memcpy(&m_registered_mac_info, (void *)registered_mac_fs_config.start_addr, sizeof(m_registered_mac_info));

    if(m_module_parameter_rebuiled_flag)
    {
        cPrintLog(CDBG_FLASH_INFO, "rebuild registered_mac!\n");
        registered_mac_rebuild_req = true;
    }
    else if(!(m_registered_mac_info.magic_top == REGISTERED_MAC_MAGIC_VAL && m_registered_mac_info.magic_bottom == REGISTERED_MAC_MAGIC_VAL)
        || (m_registered_mac_info.wifi_mac_cnt >  REGISTERED_MAC_CNT_MAX)
        || (m_registered_mac_info.ble_mac_cnt >  REGISTERED_MAC_CNT_MAX)
    )
    {
        cPrintLog(CDBG_FLASH_ERR, "r_mac not available\n");
        registered_mac_rebuild_req = true;
    }
    else if(m_registered_mac_info.crc32 != registered_mac_crc_get())
    {
        cPrintLog(CDBG_FLASH_ERR, "crc Err! registered_mac!\n");
    }

    if(!registered_mac_rebuild_req)
    {
        cPrintLog(CDBG_FLASH_DBG, "registered_mac loaded!\n");
    }
    else
    {
        cPrintLog(CDBG_FLASH_INFO, "registered_mac to default!\n");
        registered_mac_default_init();
        nrf_delay_ms(1);
        registered_mac_update(false);        
        nrf_delay_ms(10);
    }
}

bool registered_mac_update(bool reset_after_update)
{
    bool ret = false;   
    ret_code_t fs_ret;

    if(!cfg_nvm_fs_init_flag)return false;
    
    m_registered_mac_info.crc32 = registered_mac_crc_get();
    m_registered_mac_write_N_reset_flag = reset_after_update;
    // Erase one page (page 0).
    fs_ret = nrf_fstorage_erase(&registered_mac_fs_config, registered_mac_fs_config.start_addr, 1, NULL);
    if (fs_ret == NRF_SUCCESS)
    {
        fs_ret = nrf_fstorage_write(&registered_mac_fs_config, registered_mac_fs_config.start_addr, &m_registered_mac_info, sizeof(m_registered_mac_info), NULL);
        if (fs_ret == NRF_SUCCESS)
        {
            ret = true;
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "r_mac fs_store error! %d\n", fs_ret);
        }
    }
    else
    {
        cPrintLog(CDBG_FLASH_ERR, "r_mac fs_erase error! %d\n", fs_ret);
    }

    return ret;
}

bool registered_mac_get_info_wifi(uint32_t *wifi_mac_cnt, uint8_t **wifi_mac_array, uint8_t **wifi_mac_to_position_array)
{
    bool ret;
    if(((m_registered_mac_info.magic_top == REGISTERED_MAC_MAGIC_VAL) && (m_registered_mac_info.magic_bottom == REGISTERED_MAC_MAGIC_VAL))
        && m_registered_mac_info.wifi_mac_cnt > 0)
    {
        ret = true;
        if(wifi_mac_cnt)*wifi_mac_cnt=m_registered_mac_info.wifi_mac_cnt;
        if(wifi_mac_array)*wifi_mac_array=&(m_registered_mac_info.wifi_r_mac[0][0]);
        if(wifi_mac_to_position_array)*wifi_mac_to_position_array=&(m_registered_mac_info.wifi_r_position[0][0]);
    }
    else
    {
        ret = false;
        if(wifi_mac_cnt)*wifi_mac_cnt=0;
        if(wifi_mac_array)*wifi_mac_array = NULL;
        if(wifi_mac_to_position_array)*wifi_mac_to_position_array = NULL;
    }
    return ret;
}

void registered_mac_remove_all_wifi(void)
{
    m_registered_mac_info.wifi_mac_cnt = 0;
    memset(&(m_registered_mac_info.wifi_r_mac), 0, sizeof(m_registered_mac_info.wifi_r_mac));
    memset(&(m_registered_mac_info.wifi_r_position), 0xff, sizeof(m_registered_mac_info.wifi_r_position));
}

bool registered_mac_add_wifi(const uint8_t *wifi_mac, const uint8_t *wifi_position_data/*8byte position data*/)
{
    bool ret = false;
    if(wifi_mac && ((m_registered_mac_info.wifi_mac_cnt+1) <= REGISTERED_MAC_CNT_MAX))
    {
        memcpy(m_registered_mac_info.wifi_r_mac[m_registered_mac_info.wifi_mac_cnt], wifi_mac, 6);
        if(wifi_position_data)
        {
            memcpy(m_registered_mac_info.wifi_r_position[m_registered_mac_info.wifi_mac_cnt], wifi_position_data, 8);
        }
        else
        {
            memset(m_registered_mac_info.wifi_r_position[m_registered_mac_info.wifi_mac_cnt], 0xff, 8);
        }
        m_registered_mac_info.wifi_mac_cnt++;
        ret = true;
    }
    return ret;
}

bool registered_mac_get_info_ble(uint32_t *ble_mac_cnt, uint8_t **ble_mac_array, uint8_t **ble_mac_to_position_array)
{
    bool ret;
    if(((m_registered_mac_info.magic_top == REGISTERED_MAC_MAGIC_VAL) && (m_registered_mac_info.magic_bottom == REGISTERED_MAC_MAGIC_VAL))
        && m_registered_mac_info.ble_mac_cnt > 0)
    {
        ret = true;
        if(ble_mac_cnt)*ble_mac_cnt=m_registered_mac_info.ble_mac_cnt;
        if(ble_mac_array)*ble_mac_array=&(m_registered_mac_info.ble_r_mac[0][0]);
        if(ble_mac_to_position_array)*ble_mac_to_position_array=&(m_registered_mac_info.ble_r_position[0][0]);
    }
    else
    {
        ret = false;
        if(ble_mac_cnt)*ble_mac_cnt=0;
        if(ble_mac_array)*ble_mac_array=NULL;
        if(ble_mac_to_position_array)*ble_mac_to_position_array=NULL;

    }
    return ret;
}

void registered_mac_remove_all_ble(void)
{
    m_registered_mac_info.ble_mac_cnt = 0;
    memset(&(m_registered_mac_info.ble_r_mac), 0, sizeof(m_registered_mac_info.ble_r_mac));
    memset(&(m_registered_mac_info.ble_r_position), 0xff, sizeof(m_registered_mac_info.ble_r_position));
}

bool registered_mac_add_ble(const uint8_t *ble_mac, const uint8_t *ble_position_data/*8byte position data*/)
{
    bool ret = false;
    if(ble_mac && ((m_registered_mac_info.ble_mac_cnt+1) <= REGISTERED_MAC_CNT_MAX))
    {
        memcpy(m_registered_mac_info.ble_r_mac[m_registered_mac_info.ble_mac_cnt], ble_mac, 6);
        if(ble_position_data)
        {
            memcpy(m_registered_mac_info.ble_r_position[m_registered_mac_info.ble_mac_cnt], ble_position_data, 8);
        }
        else
        {
            memset(m_registered_mac_info.ble_r_position[m_registered_mac_info.ble_mac_cnt], 0xff, 8);
        }
        m_registered_mac_info.ble_mac_cnt++;
        ret = true;
    }
    return ret;
}

ssid_list_t m_ssid_list_info;

static void ssid_list_fs_evt_handler(nrf_fstorage_evt_t * p_evt);
NRF_FSTORAGE_DEF(nrf_fstorage_t ssid_list_fs_config) =
{
    .evt_handler  = ssid_list_fs_evt_handler, // Function for event callbacks.
};

static void ssid_list_fs_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        cPrintLog(CDBG_FLASH_ERR, "ssid list fs ERR! %d, %p\n", p_evt->id, (void *)p_evt->addr);
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "sl wr OK! %p\n", (void *)p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            cPrintLog(CDBG_FLASH_INFO, "sl er OK! %p\n", (void *)p_evt->addr);
        } break;

        default:
            break;
    }
}

static void ssid_list_size_check(void)
{
    if(cfg_nvm_fs_page_size < sizeof(m_ssid_list_info))  //see FS_PAGE_SIZE_WORDS
    {
        cPrintLog(CDBG_FLASH_ERR, "sl err fatal! %d, %d\n", cfg_nvm_fs_page_size, sizeof(m_ssid_list_info));
        nrf_delay_ms(100);
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);  //fs_config num_pages size over
    }
}

static void ssid_list_default_init(void)
{
    m_ssid_list_info.magic_top = SSIDLIST_MAGIC_VAL;
    m_ssid_list_info.magic_bottom = SSIDLIST_MAGIC_VAL;
    ssid_list_remove_all_white_list();
    ssid_list_remove_all_black_list();
}

static uint32_t ssid_list_crc_get(void)
{
    return crc32_compute((uint8_t*)&m_ssid_list_info, sizeof(m_ssid_list_info)-4, NULL);
}

static void ssid_list_init(void)
{
    bool ssid_list_rebuild_req = false;
    memcpy(&m_ssid_list_info, (void *)ssid_list_fs_config.start_addr, sizeof(m_ssid_list_info));

    if(ssid_list_rebuild_req)
    {
        cPrintLog(CDBG_FLASH_INFO, "rebuild ssid list!\n");
        ssid_list_rebuild_req = true;
    }
    else if(!(m_ssid_list_info.magic_top == SSIDLIST_MAGIC_VAL && m_ssid_list_info.magic_bottom == SSIDLIST_MAGIC_VAL)
    )
    {
        cPrintLog(CDBG_FLASH_ERR, "sl not available\n");
        ssid_list_rebuild_req = true;
    }
    else if(m_ssid_list_info.crc32 != ssid_list_crc_get())
    {
        cPrintLog(CDBG_FLASH_ERR, "crc Err! sl!\n");
    }

    if(!ssid_list_rebuild_req)
    {
        cPrintLog(CDBG_FLASH_DBG, "ssid_list loaded!\n");
    }
    else
    {
        cPrintLog(CDBG_FLASH_INFO, "ssid_list to default!\n");
        ssid_list_default_init();
        nrf_delay_ms(1);
        ssid_list_update();
        nrf_delay_ms(10);
    }
}

bool ssid_list_update(void)
{
    bool ret = false;   
    ret_code_t fs_ret;

    if(!cfg_nvm_fs_init_flag)return false;
    
    m_ssid_list_info.crc32 = ssid_list_crc_get();

    // Erase one page (page 0).
    fs_ret = nrf_fstorage_erase(&ssid_list_fs_config, ssid_list_fs_config.start_addr, 1, NULL);
    if (fs_ret == NRF_SUCCESS)
    {
        fs_ret = nrf_fstorage_write(&ssid_list_fs_config, ssid_list_fs_config.start_addr, &m_ssid_list_info, sizeof(m_ssid_list_info), NULL);
        if (fs_ret == NRF_SUCCESS)
        {
            ret = 0;
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "sl fs_wr err! %d\n", fs_ret);
        }
    }
    else
    {
        cPrintLog(CDBG_FLASH_ERR, "sl fs_er err! %d\n", fs_ret);
    }

    return ret;
}

void ssid_list_remove_all_white_list(void)
{
    m_ssid_list_info.ssid_white_cnt =0;
    memset(&(m_ssid_list_info.ssid_white_list), 0, sizeof(m_ssid_list_info.ssid_white_list));
}

bool ssid_list_add_white_list(const char *ssid_str)
{
    bool ret = false;
    int i;
    for(i = 0; i < SSIDLIST_ITEM_MAX; i++)
    {
        if(m_ssid_list_info.ssid_white_list[i][0] == 0)
        {
            strncpy(m_ssid_list_info.ssid_white_list[i], ssid_str, (SSIDLIST_STR_SIZE_MAX-1));
            m_ssid_list_info.ssid_white_cnt++;
            ret = true;
            cPrintLog(CDBG_MAIN_LOG, "Add wl:%s\n", ssid_str, m_ssid_list_info.ssid_white_cnt);
            break;
        }
    }
    return ret;
}

bool ssid_list_remove_white_list(unsigned int index)
{
    bool ret = false;
    int i;

    if(index < m_ssid_list_info.ssid_white_cnt)
    {
        for(i = 0; i < SSIDLIST_ITEM_MAX; i++)
        {
            if(i == (SSIDLIST_ITEM_MAX-1))
            {
                memset(m_ssid_list_info.ssid_white_list[i], 0, sizeof(m_ssid_list_info.ssid_white_list[0]));
            }
            else if(i > index)
            {
                memcpy(m_ssid_list_info.ssid_white_list[i-1], m_ssid_list_info.ssid_white_list[i], sizeof(m_ssid_list_info.ssid_white_list[0]));
            }
        }
        m_ssid_list_info.ssid_white_cnt--;
        ret = true;
    }
    return ret;
}

void ssid_list_remove_all_black_list(void)
{
    m_ssid_list_info.ssid_black_cnt = 0;
    memset(&(m_ssid_list_info.ssid_black_list), 0, sizeof(m_ssid_list_info.ssid_black_list));
}

bool ssid_list_add_black_list(const char *ssid_str)
{
    bool ret = false;
    int i;
    for(i = 0; i < SSIDLIST_ITEM_MAX; i++)
    {
        if(m_ssid_list_info.ssid_black_list[i][0] == 0)
        {
            strncpy(m_ssid_list_info.ssid_black_list[i], ssid_str, (SSIDLIST_STR_SIZE_MAX-1));
            m_ssid_list_info.ssid_black_cnt++;
            ret = true;
            cPrintLog(CDBG_MAIN_LOG, "Add bl:%s\n", ssid_str, m_ssid_list_info.ssid_black_cnt);
            break;
        }
    }
    return ret;
}

bool ssid_list_remove_black_list(unsigned int index)
{
    bool ret = false;
    int i;

    if(index < m_ssid_list_info.ssid_black_cnt)
    {
        for(i = 0; i < SSIDLIST_ITEM_MAX; i++)
        {
            if(i == (SSIDLIST_ITEM_MAX-1))
            {
                memset(m_ssid_list_info.ssid_black_list[i], 0, sizeof(m_ssid_list_info.ssid_black_list[0]));
            }
            else if(i > index)
            {
                memcpy(m_ssid_list_info.ssid_black_list[i-1], m_ssid_list_info.ssid_black_list[i], sizeof(m_ssid_list_info.ssid_black_list[0]));
            }
        }
        m_ssid_list_info.ssid_black_cnt--;
        ret = true;
    }
    return ret;
}

static void cfg_nvm_fs_flash_bounds_set(void)
{
    uint32_t base_end_addr;
    cfg_nvm_fs_page_size = NRF_FICR->CODEPAGESIZE;
#if NRF_MODULE_ENABLED(FDS)
    fds_flash_bounds_set();
    base_end_addr = fds_flash_start_addr();
#else
    base_end_addr = pstorage_flash_page_end();
#endif
    param_fs_config.end_addr = base_end_addr;
    param_fs_config.start_addr = param_fs_config.end_addr - cfg_nvm_fs_page_size;
    
    registered_mac_fs_config.end_addr = param_fs_config.start_addr;
    registered_mac_fs_config.start_addr = registered_mac_fs_config.end_addr - cfg_nvm_fs_page_size;

    ssid_list_fs_config.end_addr = registered_mac_fs_config.start_addr;
    ssid_list_fs_config.start_addr = ssid_list_fs_config.end_addr - cfg_nvm_fs_page_size;

}

static bool cfg_nvm_fs_init(void)
{
    ret_code_t fs_ret = NRF_SUCCESS;
    if(cfg_nvm_fs_init_flag)
    {
        return true;
    }
    else
    {
        cfg_nvm_fs_flash_bounds_set();
        module_parameter_size_check();
        registered_mac_size_check();
        ssid_list_size_check();
        if(fs_ret == NRF_SUCCESS)
        {
            fs_ret = nrf_fstorage_init(&param_fs_config, &nrf_fstorage_sd, NULL);
            if(fs_ret != NRF_SUCCESS)cPrintLog(CDBG_FLASH_ERR, "m_param fs_init Error! %d\n", fs_ret);
        }
        
        if(fs_ret == NRF_SUCCESS)
        {
            fs_ret = nrf_fstorage_init(&registered_mac_fs_config, &nrf_fstorage_sd, NULL);
            if(fs_ret != NRF_SUCCESS)cPrintLog(CDBG_FLASH_ERR, "r_mac fs_init Error! %d\n", fs_ret);
        }

        if(fs_ret == NRF_SUCCESS)
        {
            fs_ret = nrf_fstorage_init(&ssid_list_fs_config, &nrf_fstorage_sd, NULL);
            if(fs_ret != NRF_SUCCESS)cPrintLog(CDBG_FLASH_ERR, "sl fs_init Error! %d\n", fs_ret);
        }

        if(fs_ret == NRF_SUCCESS)
        {
#if NVM_Developer_logs
            cPrintLog(CDBG_FLASH_DBG, "param_fs_config st:%p, ed:%p, api:%p, f_info:%p\n", (void *)param_fs_config.start_addr, (void *)param_fs_config.end_addr, param_fs_config.p_api, param_fs_config.p_flash_info);
            cPrintLog(CDBG_FLASH_DBG, "registered_mac_fs_config st:%p, ed:%p, api:%p, f_info:%p\n", (void *)registered_mac_fs_config.start_addr, (void *)registered_mac_fs_config.end_addr, registered_mac_fs_config.p_api, registered_mac_fs_config.p_flash_info);
            cPrintLog(CDBG_FLASH_DBG, "ssid_list st:%p, ed:%p, api:%p, f_info:%p\n", (void *)ssid_list_fs_config.start_addr, (void *)ssid_list_fs_config.end_addr, ssid_list_fs_config.p_api, ssid_list_fs_config.p_flash_info);
#endif
            cfg_nvm_fs_init_flag = true;
            cPrintLog(CDBG_FLASH_INFO, "FlashInfo fds_start:0x%x, fds_end:0x%x\n", fds_flash_start_addr(), fds_flash_end_addr());
            cPrintLog(CDBG_FLASH_INFO, "fs OK. Param_Addr:0x%p, MAC_Addr:0x%p, SL_Addr:0x%p\n", (void *)param_fs_config.start_addr, (void *)registered_mac_fs_config.start_addr, (void *)ssid_list_fs_config.start_addr);
            return true;
        }
        else
        {
            cPrintLog(CDBG_FLASH_ERR, "fs_init Error! %d\n", fs_ret);
            cfg_nvm_fs_init_flag = false;
        }
    }
    return false;

}

void cfg_nvm_init(void)
{
#if NVM_Developer_logs
    CDBG_mask_set(CDBG_NUM2MASK(CDBG_FLASH_DBG));
#endif
    cfg_nvm_fs_init();
    module_parameter_init();
    registered_mac_init();
    ssid_list_init();
}

bool cfg_nvm_factory_reset(bool sys_reset)  //for factory reset
{
    ret_code_t fs_ret;
    if(!cfg_nvm_fs_init())
    {
        return false;
    }
    fs_ret = nrf_fstorage_erase(&param_fs_config, param_fs_config.start_addr, 1, NULL);
    if(fs_ret == NRF_SUCCESS)
    {
        fs_ret = nrf_fstorage_erase(&registered_mac_fs_config, registered_mac_fs_config.start_addr, 1, NULL);
    }
    if(fs_ret == NRF_SUCCESS)
    {
        fs_ret = nrf_fstorage_erase(&ssid_list_fs_config, ssid_list_fs_config.start_addr, 1, NULL);
    }

    if (fs_ret == NRF_SUCCESS)
    {
        if(sys_reset)
        {
            nrf_delay_ms(1000);
            cfg_board_reset();
        }
    }
    else
    {
        cPrintLog(CDBG_FLASH_ERR, "freset fs_er err! %d\n",fs_ret);
        return false;
    }
    return true;
}

bool cfg_peripheral_device_enable_status_get(peripheral_dev_type peripheral_dev)
{
    bool bEnabled = false;
    switch(peripheral_dev)
    {
        case PTYPE_SIGFOX:
            bEnabled = true;
            break;
        case PTYPE_GPS:
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = false;
            }
            else
            {
                if(m_module_parameter.gps_enable)
                {
                    bEnabled = true;
                }
            }
            break;
        case PTYPE_WIFI:
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = false;
            }
            else
            {
                if(m_module_parameter.wifi_enable)
                {
                    bEnabled = true;
                }
            }
            break;
        case PTYPE_BLE_SCAN:
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = false;
            }
            else
            {
                if(m_module_parameter.ble_beacon_scan_enable)
                {
                    bEnabled = true;
                }
            }
            break;
        case PTYPE_ACC:
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = true;
            }
            else
            {
                if(m_module_parameter.acc_operation_mode)  //0:disable, 1: shock, 2: no_motion 3: motion, 4: slope
                {
                    bEnabled = true;
                }
            }
            break;
        case PTYPE_TEMPERATURE:
#if defined(CDEV_TEMPERATURE_SENSOR_TMP102) || defined(CDEV_TEMPERATURE_SENSOR_TMP108)
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = true;
            }
            else
            {
                if(m_module_parameter.temperature_sensor_enable)
                {
                    bEnabled = true;
                }
            }
#else
            bEnabled = false;
#endif
            break;
        case PTYPE_AMBIENT_LIGHT:
#ifdef CDEV_AMBIENT_LIGHT_SENSOR
            if(m_module_parameter.operation_mode == 0/*station mode*/)
            {
                bEnabled = true;
            }
            else
            {
                if(m_module_parameter.ambient_light_sensor_enable)
                {
                    bEnabled = true;
                }
            }
#else
            bEnabled = false;
#endif
            break;
        default:
            bEnabled = false;
            break; 
            
    }
    return bEnabled;
}

