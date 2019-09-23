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

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cfg_board_LowPwr_test_func)(void);
typedef void (*cfg_board_LowPwr_Periodic_1Sec_CB_func)(void);

#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
void cfg_board_check_bootstrap_pin(void);
#endif
int cfg_board_check_bootstrap_checked_i2cdbg_connected(void);
void cfg_board_check_wifi_downloadmode(void);
void cfg_board_check_bootmode(void);
void cfg_board_testmode_BLE_Advertising_LowPwr(bool basic_resource_init, bool adv_start, cfg_board_LowPwr_test_func test_func, cfg_board_LowPwr_Periodic_1Sec_CB_func cb_func);


#ifdef __cplusplus
}
#endif
#endif // __CFG_SPECIAL_BOOT_MODE_H__

