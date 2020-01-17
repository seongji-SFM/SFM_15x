/* Copyright (c) 2018 ieTings Corp. All Rights Reserved.
 *
 * The information contained herein is property of ieTings Cor.
 * Terms and conditions of usage are described in detail in ieTings STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @brief control gps module.
 *
 * This file contains the control gps module.
 */

#ifndef __CFG_FULL_TEST_H__
#define __CFG_FULL_TEST_H__
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define CFG_FULL_TEST_MODE_TIMEOUT_SEC_CONNECT_WAIT (5 * 60)
#define CFG_FULL_TEST_MODE_TIMEOUT_SEC_RUNNING_WAIT (30 * 60)

void cfg_full_test_nus_recv_data_handler(const uint8_t * p_data, uint16_t length);
void cfg_full_test_mode_enter(void);

#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
