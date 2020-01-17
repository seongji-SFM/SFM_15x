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

#define USER_COMMAND_PARAM_SIZE_MAX 64

extern bool m_hitrun_test_flag;

extern int user_cmd_param_size;
extern uint8_t user_cmd_param_buf[USER_COMMAND_PARAM_SIZE_MAX];
extern cTBC_debugmode_run_function_t m_cTBC_dbg_mode_run_func;

void user_cmd_hitrun_input_test(void);
void user_cmd_hitrun_sense_test(void);
void user_cmd_hitrun_led_test(void);
void user_cmd_weak_symbol_for_user(void);
void dbg_i2c_user_cmd_proc(int cmd, int param_size, const uint8_t *param);

#ifdef __cplusplus
}
#endif
#endif

