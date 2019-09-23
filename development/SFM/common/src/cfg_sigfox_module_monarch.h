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
 * @brief control sigfox module.
 *
 * This file contains the control sigfox monarch module.
 */

#ifndef NRF_BLE_CFG_SIGFOX_MONARCH_H__
#define NRF_BLE_CFG_SIGFOX_MONARCH_H__
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "cfg_config_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
#define SIGFOX_SEND_PAYLOAD_SIZE (12)
#define SIGFOX_BOOT_WAIT_TICK_MONARCH (1)
typedef enum
{
    // SIGFOX_DUMMY_CMD,
    SIGFOX_OPEN_CMD,
    SIGFOX_SCAN_RC_CMD,
    SIGFOX_SET_STD_CMD,
    SIGFOX_SF_CMD,
    SIGFOX_SF_R_CMD,
    // SIGFOX_OOB_CMD,
    SIGFOX_SB_CMD,
    SIGFOX_SB_R_CMD,
    SIGFOX_SET_PUBLIC_CMD,
    // SIGFOX_POWERMODE_CMD,
    // SIGFOX_BREAK_CMD,
    SIGFOX_GET_ID_CMD,
    SIGFOX_GET_PACCODE_CMD,
    // SIGFOX_FREQUENCY_CMD,
    // SIGFOX_SNEK_CMD,
    // SIGFOX_CHECK_CHANNEL_CMD,
    // SIGFOX_RESET_CHANNEL_CMD,
    SIGFOX_SET_POWERLEVEL_CMD,
    // SIGFOX_SAVE_CONFIG_CMD,
    SIGFOX_STOP_MESSAGE_CMD,
    // SIGFOX_RX_FREQUENCY_CMD,
    SIGFOX_SET_ENCRIPTION_CMD,
    SIGFOX_SET_PA_CMD,
    SIGFOX_SET_RSSI_CMD,
    SIGFOX_SET_LBT_CMD,
} sigfox_cmd_c;
/**@brief Structure for sigfox state machine. */
typedef enum
{
    NONE_S,
    SETUP_S,
    INIT_S,
    INIT_R,
    OPEN_S,
    OPEN_R,
    SET_STD_CFG_S,
    SET_STD_CFG_R,
    SW_PUBLIC_S,
    SW_PUBLIC_R,
    SET_PAY_ENC_S,
    SET_PAY_ENC_R,
    SW_PA_S,
    SW_PA_R,
    // CHECK_CHANNEL_S,
    // CHECK_CHANNEL_R,
    // RESET_CHANNEL_S,
    // RESET_CHANNEL_R,
    TRANSMIT_FRAME_S,
    TRANSMIT_FRAME_R,
    TRANSMIT_FRAME_DOWNLINK_S,
    TRANSMIT_FRAME_DOWNLINK_R,
    TRANSMIT_BIT_S,
    TRANSMIT_BIT_R,
    TRANSMIT_BIT_DOWNLINK_S,
    TRANSMIT_BIT_DOWNLINK_R,
    // TRANSMIT_OOB_S,
    // TRANSMIT_OOB_R,
    IDLE_S,
    IDLE_R,
    BYPASS_INIT,  //FEATURE_CFG_BYPASS_CONTROL
    BYPASS_WORK,  //FEATURE_CFG_BYPASS_CONTROL
    GET_ID_S,
    GET_ID_R,
    GET_PACCODE_S,
    GET_PACCODE_R,
    // GET_FREQUENCY_S,
    // GET_FREQUENCY_R,
    SET_POWERLEVEL_S,
    SET_POWERLEVEL_R,
    // SAVE_CONFIG_S,
    // SAVE_CONFIG_R,
    SAVE_DOWNLINK_S,
    SAVE_DOWNLINK_R,
    // GET_RX_FREQUENCY_S,
    // GET_RX_FREQUENCY_R,
    SCAN_RC_S,
    SCAN_RC_R,
    SET_RSSI_S,
    SET_RSSI_R,
    SET_LBT_S,
    SET_LBT_R,
    EXIT
} sigfox_state_s;

typedef enum
{
    RCZ_1,
    RCZ_2,
    RCZ_3,
    RCZ_4
} sigfox_rcz;

typedef void (*sigfox_bypass_recv_handler_t)(const uint8_t *p_data, uint32_t data_size);
typedef void (*sigfox_bypass_noti_callback_t)(bool is_ok, const char *noti_str);

extern uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal
extern bool scan_rc_parameter;
extern bool sigfox_rc_checked;
extern bool sigfox_transmit_fail;

/**
 * @brief function for powering on sigfox module 
 */

void sigfox_power_on(bool on_off);

/**
 * @brief function for starting  sigfox module timer.
 * 
 */
void cfg_sigfox_timers_start(void);

/**
 * @brief function for stoping  sigfox module timer.
 * 
 */
void cfg_sigfox_timers_stop(void);

/**
 * @brief function for getting  the current state from sigfox module.
 * 
 * @return current state from sigfox module
 */
sigfox_state_s sigfox_get_state(void);

/**
 * @brief function for indication whether exit state in sigfox module is or not.
 * 
 * @return true if the current state is exit state, nor false
 */

bool sigfox_check_exit_excute(void);


/**@brief Function for setting sigfox state in sigfox scheduler.
 *
 * @param[in] state  set the next state in sigfox scheduler.
 *                         
 */

void sigfox_set_state(sigfox_state_s m_state);
void sigfox_send_payload(uint8_t * send_data, uint32_t send_data_size, uint8_t ** received_data);  //This function should be called from main().

#ifdef FEATURE_CFG_BYPASS_CONTROL

/**
 * @brief       Function for bypass mode
 *
 * @return      CWIFI_Result_OK on success. Otherwise an error code(eg.CWIFI_Result_Busy).
 * @param[callback function]   recv_CB sigfox recv data callback
 * @param[callback function]   noti_CB state notification
 */
int sigfox_bypass_req(sigfox_bypass_recv_handler_t recv_CB, sigfox_bypass_noti_callback_t noti_CB);

/**@brief Function for indicating whether the current mode is test mode or not.
 *
 * @return true if the current mode is test mode, nor false
 *                         
 */

bool sigfox_is_bypass_mode(void);
/**
 * @brief       Function for bypass data to sigfox module
 * @param[in]   pData pointer of send data
 * @param[in]   dataSize size of send data
 *
 * @return      boolean result of data send request to wifi module
 */
bool sigfox_bypass_write_request(const char *p_data, unsigned int data_size);
#endif
/**@brief Function for downlink on off
 *
 * @param[in] on off flag
 * @return old setting value
 *                         
 */
bool cfg_sigfox_downlink_on_off(bool on_off);

/**@brief Function for senk_testmode_enable
 *
 * @param[in] enable flag
 * @return old setting value
 *                         
 */
bool cfg_sigfox_set_senk_testmode_enable(bool enable);

/**@brief Function for setting flag to change scenario mode to test mode.
 *
 *                         
 */

void sigfox_abort_request(void);



/**@brief Function for gettting sigfox ID and PAK.
 *
 *                         
 */

void cfg_sigfox_prepare_start(void);

#endif
#ifdef __cplusplus
}
#endif

#endif // NRF_BLE_CFG_MAIN_H__

/** @} */

