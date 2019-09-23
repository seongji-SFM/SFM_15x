/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
*
* The information contained herein is property of WISOL Cor.
* Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
*
* This heading must NOT be removed from
* the file.
*
*/
#ifndef __CFG_USER_CMD_PROC_H__
#define __CFG_USER_CMD_PROC_H__
#ifdef __cplusplus
extern "C" {
#endif

extern bool m_hitrun_test_flag;

void dbg_i2c_user_cmd_proc(int cmd, int param_size, const uint8_t *param);

#ifdef __cplusplus
}
#endif
#endif

