/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __CFG_SPECIAL_BOOT_MODE_H__
#define __CFG_SPECIAL_BOOT_MODE_H__
#include "cfg_config_defines.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMODE_REG_GPREGRET_MASK      (0xF8) //see BOOTLOADER_DFU_GPREGRET_MASK            (0xF8)
#define BMODE_REG_GPREGRET           (0x48) //see BOOTLOADER_DFU_GPREGRET                 (0xB0), if (nrf_power_gpregret_get() & BOOTLOADER_DFU_START)

#define BMODE_REG_MODE_BIT_MASK                 (0x07) //see BOOTLOADER_DFU_START_BIT_MASK           (0x01)
#define BMODE_REG_MODE_FACTORY_TEST_WITH_BLE    (0x00) //see BOOTLOADER_DFU_GPREGRET                 (0xB0), if (nrf_power_gpregret_get() & BOOTLOADER_DFU_START)
#define BMODE_REG_MODE_ENTER_DEEP_SLEEP         (0x02) //see BOOTLOADER_DFU_GPREGRET                 (0xB0), if (nrf_power_gpregret_get() & BOOTLOADER_DFU_START)
#define BMODE_REG_MODE_USER_DEFINED             (0x04) //see BOOTLOADER_DFU_GPREGRET                 (0xB0), if (nrf_power_gpregret_get() & BOOTLOADER_DFU_START)

typedef enum
{
    bmode_reg_none,
    bmode_reg_factory_test_with_ble,
    bmode_reg_enter_deep_sleep,
    bmode_reg_user_defined,  //bmode_reg_user_def_cb
    bmode_reg_max
}boot_mode_in_reg_e;

typedef void (*cfg_board_LowPwr_test_func)(void);
typedef void (*cfg_board_LowPwr_Periodic_1Sec_CB_func)(void);
typedef void (*cfg_bmode_reg_user_defined_func)(void);  //return value : reset request

extern cfg_bmode_reg_user_defined_func bmode_reg_user_def_cb;  //return value : reset request

#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
void cfg_board_check_bootstrap_pin(void);
#endif
int cfg_board_check_bootstrap_checked_i2cdbg_connected(void);
void cfg_board_check_wifi_downloadmode(void);
void cfg_board_check_bootmode_in_flash(void);
void cfg_board_check_bootmode_in_register(void);
void cfg_board_set_bootmode_in_register_with_ble(boot_mode_in_reg_e mode);
void cfg_board_testmode_BLE_Advertising_LowPwr(bool basic_resource_init, bool adv_start, cfg_board_LowPwr_test_func test_func, cfg_board_LowPwr_Periodic_1Sec_CB_func cb_func);


#ifdef __cplusplus
}
#endif
#endif // __CFG_SPECIAL_BOOT_MODE_H__

