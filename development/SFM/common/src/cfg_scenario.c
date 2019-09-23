/* Copyright (c) 2019 WISOL Corp. All Rights Reserved.
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"

#include "cfg_config.h"
#include "cfg_scenario.h"
#include "cfg_dbg_log.h"

int main_wakeup_reason = main_wakeup_reason_normal;  //main_wakeup_reason_type
bool main_wakeup_interrupt;
bool test_nus_full_tracking_mode = false;
bool nus_disconnect_reset = false;
bool main_powerdown_request = false;
int m_module_waitMS_N_powerdown_req = 0;
bool nus_disconnect_powerdown = false;
static bool main_powerdown_msg_send = false;

extern module_mode_t m_module_mode;
extern void main_set_module_state(module_mode_t state);
extern module_mode_t main_get_module_state_next(bool normal);
extern void main_timer_schedule_restart_check_idle(void);
extern void sigfox_power_on(bool on);
extern void main_send_poweroff_msg(void);

void cfg_scen_wakeup_request(main_wakeup_reason_type reason)
{
    if(cfg_scen_check_sleep_state())
    {
        main_wakeup_interrupt = true;
        main_wakeup_reason = reason;
        cPrintLog(CDBG_FCTRL_INFO, "wakeup req! reason:%d\n", reason);
    }
    else
    {
        cPrintLog(CDBG_FCTRL_INFO, "skip wakeup req! cur state:%d, old reason:%d, new reson:%d\n", m_module_mode, main_wakeup_reason, reason);
    }
}
void cfg_scen_powerdown_request(int delayMS, bool send_poweroff_msg)
{
    main_powerdown_request = true;    
    m_module_waitMS_N_powerdown_req = delayMS;
    main_powerdown_msg_send = send_poweroff_msg;
}

bool cfg_scen_check_sleep_state(void)
{
    return (!main_wakeup_interrupt && main_schedule_state_is_idle());
}

void cfg_scen_init(void)
{
    main_wakeup_interrupt = false;
    main_set_module_state(main_get_module_state_next(true));
}

void cfg_scen_main_handler(void)
{
    if(main_wakeup_interrupt == true)
    {
        main_wakeup_interrupt = false;
        main_timer_schedule_restart_check_idle();
    }

    if(main_powerdown_request)
    {
        cPrintLog(CDBG_MAIN_LOG, "proc powerdown(delay%d)\n", m_module_waitMS_N_powerdown_req);
        if(m_module_waitMS_N_powerdown_req)
            nrf_delay_ms(m_module_waitMS_N_reset_req);
        main_powerdown_request = false;
#ifdef USR_MODULE_GPIO_DEF_BUTTON
        cfg_button_release();
#endif

        if(main_powerdown_msg_send)
        {
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
            main_send_poweroff_msg();
#endif
        }
        app_timer_stop_all();
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
        sigfox_power_on(false);
#endif
        cfg_board_indicate_power_down();

#ifdef USR_MODULE_GPIO_DEF_BUTTON_POWER_OFF
        cPrintLog(CDBG_MAIN_LOG, "wait power key release\n");
        nrf_gpio_cfg_input(USR_MODULE_GPIO_DEF_BUTTON, NRF_GPIO_PIN_PULLDOWN);
        while(nrf_gpio_pin_read(USR_MODULE_GPIO_DEF_BUTTON) == 1);
#endif
        cfg_board_prepare_power_down();
        cPrintLog(CDBG_MAIN_LOG, "power off\n");
        cfg_board_goto_power_down();
    }

}

