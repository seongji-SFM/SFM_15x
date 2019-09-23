/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */
#ifndef __CFG_BOARD_WIFI_TX_POWER_TABLES_H__
#define __CFG_BOARD_WIFI_TX_POWER_TABLES_H__

#include "cfg_config_defines.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifdef CDEV_WIFI_MODULE
void cfg_board_check_update_wifi_tx_pwr_tables(void);
#else
#define cfg_board_check_update_tx_pwr_tables()
#endif
#ifdef __cplusplus
}
#endif
#endif // __CFG_BOARD_WIFI_TX_POWER_TABLES_H__
