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
 * @brief tracking Sample Application cfg_sigfox_module.c file.
 *
 * This file contains the source code for an tracking sample application.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
#include "nrf_delay.h"
#include "cfg_config.h"
#include <ctype.h>
#include "cfg_sigfox_module.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"

#include "cfg_twis_board_control.h"
#include "cfg_nvm_ctrl.h"

#if defined(CDEV_SIGFOX_MONARCH_MODULE) //temp for CDEV_SIGFOX_MONARCH_MODULE

#define SIGFOX_STATE_MACHINE_TICK_MS 200
#define SIGFOX_DELAY_FOR_UART_INIT 200
#define SIGFOX_DELAY_FOR_READY 200

// #define SIGFOX_DUMMY_COMMAND                "help\r\n"
            // FIXME scan_rc check time
#ifndef DEVELOP_TIME_TEST
#define SIGFOX_START_RCZ_SCAN_COMMAND       "node_execute_monarch_scan 0x3F 6 2"
#else
#define SIGFOX_START_RCZ_SCAN_COMMAND       "node_execute_monarch_scan 0x3F 1 1"
#endif
#define SIGFOX_STOP_RCZ_SCAN_COMMAND        "node_stop_monarch_scan"
#define SIGFOX_RCZ_OPEN_COMMAND             "node_open_with_zone"
#define SIGFOX_RCZ_STD_CONFIG_COMMAND       "node_set_std_config"
#define SIGFOX_SF_COMMAND                   "node_send_frame"
#define SIGFOX_SB_COMMAND                   "node_send_bit"
#define SIGFOX_SWITCH_PUBLIC_KEY_COMMAND    "switch_public_key"
#define SIGFOX_GET_INFO_COMMAND             "node_get_info"
#define SIGFOX_GET_VERSION_COMMAND          "node_get_version"
#define SIGFOX_GET_ID_COMMAND               "get_id"
#define SIGFOX_GET_PAC_COMMAND              "get_pac"
#define SIGFOX_GET_LIB_VERSION_COMMAND      "get_lib_version"
#define SIGFOX_SET_PAYLOAD_ENCRIPTION_COMMAND   "set_payload_encryption"
#define SIGFOX_REDUCE_POWERLEVEL_COMMAND       "reduce_output_power"
#define SIGFOX_SET_PA_COMMAND       "switch_pa"
#define SIGFOX_STOP_MESSAGE_COMMAND "node_send_frame {01} 2 0"
#define SIGFOX_SET_RSSI_COMMAND     "set_rssi_offset 0"
#define SIGFOX_SET_LBT_COMMAND     "set_lbt_thr_offset 11"


// #define RC2_SET_STD_CONFIG_WORD_0          (uint32_t)0x000001FF
#define RC2_SET_STD_CONFIG_WORD_0          (uint32_t)0x00000001
#define RC2_SET_STD_CONFIG_WORD_1          (uint32_t)0x00000000
#define RC2_SET_STD_CONFIG_WORD_2          (uint32_t)0x00000000
#define RC2_SET_STD_TIMER                  (uint8_t)1

#define RC3_SET_STD_CONFIG_WORD_0          (uint32_t)0x00000003
// #define RC3_SET_STD_CONFIG_WORD_1          (uint32_t)0x00001F40
#define RC3_SET_STD_CONFIG_WORD_1          (uint32_t)0x00001388
// #define RC3_SET_STD_CONFIG_WORD_2          (uint32_t)0x0000009B
#define RC3_SET_STD_CONFIG_WORD_2          (uint32_t)0x00000000
#define RC3_SET_STD_TIMER                  (uint8_t)0

#define RC4_SET_STD_CONFIG_WORD_0          (uint32_t)0x00000000
#define RC4_SET_STD_CONFIG_WORD_1          (uint32_t)0x40000000
#define RC4_SET_STD_CONFIG_WORD_2          (uint32_t)0x00000000
#define RC4_SET_STD_TIMER                  (uint8_t)1

#define RC5_SET_STD_CONFIG_WORD_0          (uint32_t)0x00000003
// #define RC5_SET_STD_CONFIG_WORD_1          (uint32_t)0x00001F40
#define RC5_SET_STD_CONFIG_WORD_1          (uint32_t)0x00001388
// #define RC5_SET_STD_CONFIG_WORD_2          (uint32_t)0x0000009B
#define RC5_SET_STD_CONFIG_WORD_2          (uint32_t)0x00000000
#define RC5_SET_STD_TIMER                  (uint8_t)0

#define UPLINK_REPEAT   (uint8_t)2
#define PAYLOAD_ENCRIPTION_ENABLE   0

sigfox_state_s m_sigfox_state;
sigfox_cmd_c m_sigfox_cmd;
bool m_sigfox_state_init_flag;
bool m_sigfox_abort_flag;
unsigned int m_sigfox_proc_counter;

bool sigfox_rc_checked = false;
bool sigfox_transmit_fail = false;

bool received_ok;
bool received_fail;
bool data_received;
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define CFG_UART_TIMEOUT                50

#define ONE_DAY_SEC (60*60*24)

#define SFX_SETUP_TIMEOUT    5*2               // 2sec
#define SFX_TRANSMIT_TIMEOUT    5*25                // 25 Sec
#define SFX_TRANSMIT_DN_TIMEOUT 5*60      // 1min
#define SFX_RC_SCAN_TIMEOUT     5*(60*5+12)      // 5min 12sec

APP_TIMER_DEF(m_sigfox_timer_id);                        /**SIGFOX timer. */
uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal

uint8_t downlink_data[40];
uint8_t downlink_data_size;
uint8_t downlink_convert[8];

volatile bool m_get_parameter = false;
volatile bool m_set_parameter = false;
bool scan_rc_parameter = false;

bool m_downlink_msg_on = false;
bool m_snek_testmode_enable = false;
int m_powerlevel = 0;       // 1 : -0.5dB
bool m_save_result = false;

extern uint32_t downlink_max;

extern bool scan_wifi_cnt;
extern bool mSTOPmessage;
extern bool mWifimsg_send;
static sigfox_bypass_recv_handler_t cb_cfg_sigfox_bypass_recv;
static sigfox_bypass_noti_callback_t cb_cfg_sigfox_bypass_noti;

static void cfg_sigfox_uart_init(void);
static void cfg_sigfox_uart_uninit(void);
static void cfg_sigfox_timer_create(void);

uint8_t hextonum(uint8_t c)
{
    if( '0' <= c && c <= '9' )
        return c - '0';

    if( 'a' <= c && c <= 'f' )
        return c - 'a' + 10;

    if( 'A' <= c && c <= 'F' )
        return c - 'A' + 10;

    return 0;
}

void sigfox_hexdigit_to_hexnum(uint8_t* dec, uint8_t* src, uint8_t size)
{
    uint8_t dec_start;
    uint8_t src_start;

    for(src_start=0,dec_start=0 ; src_start < size ;)
    {
        dec[dec_start] = hextonum(src[src_start++])<<4;
        dec[dec_start++] |=hextonum(src[src_start++]); 
    }
}

void cfg_sigfox_prepare_start(void)
{
    m_get_parameter = true;
    cfg_sigfox_timer_create();
    sigfox_set_state(SETUP_S);
    cfg_sigfox_timers_start();
    while(m_get_parameter);
    cfg_sigfox_timers_stop();
    sigfox_set_state(SETUP_S);
}

//power on delay tuning
static bool sigfox_power_on_old_status = false;
static int sigfox_power_on_step_settup_s = 0;

static void sigfox_power_uart_to_low(void)
{
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_UART_TX);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_UART_TX, 0);
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_UART_RX);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_UART_RX, 0);
}

static void __sigfox_power_off(void)
{
    //bus release
    cfg_sigfox_uart_uninit();
    
    //bus line to low
    sigfox_power_uart_to_low();
    nrf_delay_ms(1);
    
    //power enable pin and reset pin to low
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 0);
//    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
    cfg_board_common_power_control(module_comm_pwr_sigfox, false);
}
//power on delay tuning

void sigfox_power_on(bool on)
{
    if(on != sigfox_power_on_old_status)
    {
        cPrintLog(CDBG_SIGFOX_INFO, "%s : %d\n", __func__, on);
        if(on)
        {
            //bus line to low
            sigfox_power_uart_to_low();

            //power enable pin control   
            cfg_board_common_power_control(module_comm_pwr_sigfox, true);
            nrf_delay_ms(2);
            nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
            nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
            nrf_delay_ms(5);  //spec is 4ms
//            nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//            nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
            nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
            nrf_delay_ms(10);
//            nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
            nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);

            nrf_delay_ms(SIGFOX_DELAY_FOR_UART_INIT);
            //bus init
            cfg_sigfox_uart_init();
            nrf_delay_ms(SIGFOX_DELAY_FOR_READY);
        }
        else
        {
            __sigfox_power_off();
            sigfox_power_on_step_settup_s = 0;
        }
        sigfox_power_on_old_status = on;
    }
    else
    {
        if(!on && (sigfox_power_on_step_settup_s > 3))  //power on delay tuning
        {
            __sigfox_power_off();
            sigfox_power_on_step_settup_s = 0;
            sigfox_power_on_old_status = false;
        }
        else
        {
            cPrintLog(CDBG_SIGFOX_INFO, "%s pwr ctrl skip: old:%d, not:%d\n", __func__, on);
        }
    }
}


sigfox_state_s sigfox_get_state(void)
{
    return m_sigfox_state;
}

bool sigfox_check_exit_excute(void)
{
    bool ret=false;
    if((m_sigfox_state == EXIT) && (m_sigfox_state_init_flag == false))
        ret = true;
    else
        ret = false;
    return ret;
}

void sigfox_set_state(sigfox_state_s m_state)
{
    m_sigfox_state = m_state;
    if(m_state == BYPASS_INIT)
    {
        m_sigfox_state_init_flag = true;
    }
}

void sigfox_send_payload(uint8_t * send_data, uint32_t send_data_size, uint8_t ** received_data)
{
    sigfox_set_state(SETUP_S);

    // set flag to decide to receive downlink
    if(received_data)
        cfg_sigfox_downlink_on_off(true);
    else
        cfg_sigfox_downlink_on_off(false);

    // copy user data to sending buffer
    cfg_bin_2_hexadecimal(send_data, (SIGFOX_SEND_PAYLOAD_SIZE>send_data_size)?send_data_size:SIGFOX_SEND_PAYLOAD_SIZE, (char *)frame_data);

    // start sigfox module timer
    cfg_sigfox_timers_start();

    // check whether sigfox module finishes
    while(!sigfox_check_exit_excute());

    // stop sigfox module timer
    cfg_sigfox_timers_stop();
    if(received_data)
        *received_data = downlink_data;
}

#ifdef FEATURE_CFG_BYPASS_CONTROL
int sigfox_bypass_req(sigfox_bypass_recv_handler_t recv_CB, sigfox_bypass_noti_callback_t noti_CB)
{
    sigfox_state_s sigfox_state;
    sigfox_state = sigfox_get_state();
    if(sigfox_state == SETUP_S || sigfox_state == NONE_S)
    {
        sigfox_set_state(BYPASS_INIT);
        cfg_sigfox_timers_start();
        cb_cfg_sigfox_bypass_recv = recv_CB;
        cb_cfg_sigfox_bypass_noti = noti_CB;
        return NRF_SUCCESS;
    }
    return NRF_ERROR_BUSY;
}

bool sigfox_is_bypass_mode(void)
{
    if(m_sigfox_state == BYPASS_WORK)
        return true;
    else
        return false;
}


bool sigfox_bypass_write_request(const char *p_data, unsigned int data_size)
{
    if(m_sigfox_state == BYPASS_WORK)
    {
        int i;
        for(i=0; i<data_size; i++)
        {
            app_uart_put(*p_data++);
        }
        return NRF_SUCCESS;
    }
    return NRF_ERROR_INVALID_STATE;
}
#endif

void sigfox_abort_request(void)
{
    cPrintLog(CDBG_SIGFOX_INFO, "%s old state : %d\n", __func__, m_sigfox_state);
#if 1 //def FEATURE_CFG_BYPASS_CONTROL
    m_sigfox_abort_flag = true;
#endif
}

static uint8_t send[128];
void sigfox_Send_Command(void) 
{
    volatile uint32_t err_code;
    uint16_t length, count=0;

    memset(send, 0x00,128 );
    switch( m_sigfox_cmd ) 
    {
        case SIGFOX_OPEN_CMD: 
            sprintf((char*)send,"%s %d\r\n",SIGFOX_RCZ_OPEN_COMMAND,module_parameter_get_val(module_parameter_item_sigfox_RC_number)) ; 
            break;
        case SIGFOX_SCAN_RC_CMD:
            sprintf((char*)send,"%s\r\n",SIGFOX_START_RCZ_SCAN_COMMAND);
            break;
        case SIGFOX_SET_STD_CMD:
        {
            switch(module_parameter_get_val(module_parameter_item_sigfox_RC_number)){
                case 1: 
                    return;
                case 2: 
                    sprintf((char*)send,"%s 0x%X 0x%X 0x%X %d\r\n",SIGFOX_RCZ_STD_CONFIG_COMMAND,RC2_SET_STD_CONFIG_WORD_0,RC2_SET_STD_CONFIG_WORD_1,RC2_SET_STD_CONFIG_WORD_2,RC2_SET_STD_TIMER); 
                break;
                case 3: 
                    sprintf((char*)send,"%s 0x%X 0x%X 0x%X %d\r\n",SIGFOX_RCZ_STD_CONFIG_COMMAND,RC3_SET_STD_CONFIG_WORD_0,RC3_SET_STD_CONFIG_WORD_1,RC3_SET_STD_CONFIG_WORD_2,RC3_SET_STD_TIMER); 
                break;
                case 4: 
                    sprintf((char*)send,"%s 0x%X 0x%X 0x%X %d\r\n",SIGFOX_RCZ_STD_CONFIG_COMMAND,RC4_SET_STD_CONFIG_WORD_0,RC4_SET_STD_CONFIG_WORD_1,RC4_SET_STD_CONFIG_WORD_2,RC4_SET_STD_TIMER); 
                break;
                case 5: 
                    sprintf((char*)send,"%s 0x%X 0x%X 0x%X %d\r\n",SIGFOX_RCZ_STD_CONFIG_COMMAND,RC5_SET_STD_CONFIG_WORD_0,RC5_SET_STD_CONFIG_WORD_1,RC5_SET_STD_CONFIG_WORD_2,RC5_SET_STD_TIMER); 
                break;
                case 6: 
                    return;
                default:
                    return;
            }
        }
        break;
        case SIGFOX_SF_CMD : 
            sprintf((char*)send,"%s {%s} %d 0\r\n", SIGFOX_SF_COMMAND,frame_data,UPLINK_REPEAT); 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send frame request %s", send);
            break ;  
        case SIGFOX_SF_R_CMD :
            sprintf((char*)send,"%s {%s} %d 1\r\n", SIGFOX_SF_COMMAND,frame_data,UPLINK_REPEAT); 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send frame request %s", send);
            break;
        case SIGFOX_SB_CMD : 
            sprintf((char*)send,"%s %s %d 0\r\n", SIGFOX_SB_COMMAND,frame_data,UPLINK_REPEAT); 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send bit request %s", send);
            break ;  
        case SIGFOX_SB_R_CMD :
            sprintf((char*)send,"%s %s %d 1\r\n", SIGFOX_SB_COMMAND,frame_data,UPLINK_REPEAT); 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send bit request %s", send);
            break;
        // case SIGFOX_OOB_CMD:
            // break;
        case SIGFOX_SET_PUBLIC_CMD:
            sprintf((char*)send,"%s 1\r\n",SIGFOX_SWITCH_PUBLIC_KEY_COMMAND); 
            break;
        // case SIGFOX_POWERMODE_CMD : 
            // sprintf((char*)send, SIGFOX_POWERMODE_COMMAND) ; 
            // break;
        // case SIGFOX_BREAK_CMD:
        // sprintf((char*)send, SIGFOX_BREAK_COMMAND);
        // break;
        // case SIGFOX_SNEK_CMD:
        // sprintf((char*)send, SIGFOX_SNEK_COMMAND);
        // break;
        case SIGFOX_GET_ID_CMD:
            sprintf((char*)send,"%s\r\n",SIGFOX_GET_ID_COMMAND);
            break;
        case SIGFOX_GET_PACCODE_CMD:
            sprintf((char*)send,"%s\r\n",SIGFOX_GET_PAC_COMMAND);
            break;
        // case SIGFOX_CHECK_CHANNEL_CMD:
        // sprintf((char*)send, SIGFOX_CHECK_CHANNEL_COMMAND);
        // break;
        // case SIGFOX_RESET_CHANNEL_CMD:
        // sprintf((char*)send, SIGFOX_RESET_CHANNEL_COMMAND);
        // break;
        // case SIGFOX_FREQUENCY_CMD:
        // sprintf((char*)send, SIGFOX_FREQUENCY_COMMAND);
        // break;
        case SIGFOX_SET_POWERLEVEL_CMD:
            sprintf((char*)send,"%s %d\r\n",SIGFOX_REDUCE_POWERLEVEL_COMMAND,m_powerlevel);
            break;
        // case SIGFOX_SAVE_CONFIG_CMD:
        // sprintf((char*)send, SIGFOX_SAVE_CONFIG_COMMAND);
        // break;
        case SIGFOX_STOP_MESSAGE_CMD : 
            sprintf((char*)send,"%s\r\n",SIGFOX_STOP_MESSAGE_COMMAND) ;
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send request %s", send);
            break;
        // case SIGFOX_RX_FREQUENCY_CMD:
        // sprintf((char*)send, SIGFOX_RX_FREQUENCY_COMMAND);
        // break;
        case SIGFOX_SET_ENCRIPTION_CMD:
            sprintf((char*)send,"%s 1\r\n",SIGFOX_SET_PAYLOAD_ENCRIPTION_COMMAND);
            break;
        case SIGFOX_SET_PA_CMD:
            sprintf((char*)send,"%s 1\r\n",SIGFOX_SET_PA_COMMAND);
            break;
        case SIGFOX_SET_RSSI_CMD:
            sprintf((char*)send,"%s\r\n",SIGFOX_SET_RSSI_COMMAND);
            break;
        case SIGFOX_SET_LBT_CMD:
            sprintf((char*)send,"%s\r\n",SIGFOX_SET_LBT_COMMAND);
            break;
        // case SIGFOX_POWERMODE_CMD : 
        default:
            return;

    }
    cPrintLog(CDBG_SIGFOX_DBG,"** send command : %s\n",send);
    length = strlen((char*)send);
    while (count < length)
    {
        err_code = app_uart_put(send[count++]);
    }
    (void)err_code;
 }

bool cfg_sigfox_check_channel(uint8_t * received_data)
{
    uint8_t temp[3];
    int rsize, tsize, rvalue;

    memset(temp,0x00,sizeof(temp));

    cPrintLog(CDBG_SIGFOX_INFO, "channel: %s\n",received_data);
    for(tsize = 0,rsize = 0; rsize < downlink_data_size;rsize++)
    {
        if(downlink_data[rsize]!=',')
        {
            temp[tsize++]=downlink_data[rsize];
        }
        else
        {
            rvalue= atoi((char*)temp);
            if(rvalue==0)
                return true;
            memset(temp,0x00,sizeof(temp));
            tsize = 0;
        }
    }
    rvalue= atoi((char*)temp);

    if(rvalue<3)
        return true;

    return false;
}
void sigfox_get_ap_key(uint8_t * received_data)
{
    uint8_t temp[16];
    uint8_t s_len,k_len;

    for(s_len = 0, k_len = 0; s_len<downlink_data_size ; s_len++){
        if(downlink_data[s_len] == 'x'){
            temp[k_len++] = downlink_data[++s_len];
            temp[k_len++] = downlink_data[++s_len];
        } 
    }
    /*
    for(s_len = 0, k_len=0; s_len < downlink_data_size;s_len++)
    {
        if(downlink_data[s_len]!=' ')
        {
            temp[k_len++]=downlink_data[s_len];
        }
    }
    */
    sigfox_hexdigit_to_hexnum(received_data, temp, k_len);
}

void sigfox_received_data(uint8_t * received_data, uint8_t length)
{
    bool sfx_error_check=false;
    bool sfx_return_check=false;
    const char sfx_err_str[] = "sfx_error:";
    const uint8_t sfx_err_str_len = 10;

    if(CDBG_mask_val_get() & CDBG_SIGFOX_DBG)
    {
        char recv_log_data[256];
        int idx;
        if((length+1)>=(sizeof(recv_log_data)-1))
        {
            idx = (sizeof(recv_log_data)-1);
        }
        else
        {
            idx = (length+1);
        }
        memcpy(recv_log_data, received_data, idx);
        recv_log_data[idx] = 0;
        cPrintLog(CDBG_SIGFOX_DBG,"** Recv Resp : %s\n", recv_log_data);
    }
    if(0){}  //dummy if
#ifdef FEATURE_CFG_BYPASS_CONTROL
    else if(m_sigfox_state == BYPASS_WORK)
    {
        if(length)
        {
            if(cb_cfg_sigfox_bypass_recv)cb_cfg_sigfox_bypass_recv(received_data, length);
        }
    }
#endif
    else
    {
        uint8_t *search_str_pos;
        uint8_t downlink_str_len;
        uint8_t search_str[20]={0,};
        uint8_t search_str_len;

        switch(m_sigfox_cmd){
            case SIGFOX_GET_ID_CMD:
                {
                    char str[] = "{id:";
                    downlink_str_len = 8;
                    sfx_return_check = true;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
                }
                break;
            case SIGFOX_GET_PACCODE_CMD:
                {
                    char str[] = "{pac: ";
                    downlink_str_len = 16;
                    sfx_return_check = true;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
                }
                break;
            case SIGFOX_SF_R_CMD:
            case SIGFOX_SB_R_CMD:
                {
                    char str[] = "customer_resp: ";
                    downlink_str_len = 39;
                    sfx_return_check = true;
                    sfx_error_check = true;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
                }
                break;
            case SIGFOX_SET_STD_CMD:
            case SIGFOX_SET_ENCRIPTION_CMD:
            case SIGFOX_SF_CMD:
            case SIGFOX_SB_CMD:
            case SIGFOX_STOP_MESSAGE_CMD:
            case SIGFOX_SET_PUBLIC_CMD:
            case SIGFOX_SET_POWERLEVEL_CMD:
            case SIGFOX_OPEN_CMD:
            case SIGFOX_SET_RSSI_CMD:
            case SIGFOX_SET_LBT_CMD:
                {
                    // char str[] = "sfx_error:";
                    sfx_error_check = true;
                    downlink_str_len = 0;
                    // search_str_len = strlen(str);
                    // memcpy(search_str, (uint8_t *)str, search_str_len);
                }
                break;
            case SIGFOX_SET_PA_CMD:
                {
                    char str[] = "API call...";
                    downlink_str_len = 0;
                    sfx_return_check = true;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
                }
                break;
            case SIGFOX_SCAN_RC_CMD:
                {
#if 0       // Detected RC check 
                    char str[] = "Detected RC";
                    downlink_str_len = 1;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
#else
                    char str[] = "rc_bit_mask ";
                    downlink_str_len = 2;
                    sfx_return_check = true;
                    sfx_error_check = true;
                    search_str_len = strlen(str);
                    memcpy(search_str, (uint8_t *)str, search_str_len);
#endif
                }
                break;
        }

        // cPrintLog(CDBG_SIGFOX_DBG,"** received_data:%s\n",received_data);


#if 0
        if(sfx_error_check){
            if (search_str_pos && !received_ok){
                uint8_t *search_str_end_pos;
                uint8_t i;

                search_str_end_pos = (uint8_t *)strchr((const char *)search_str_pos, (int)0x7d);     // } search

                // sigfox_hexdigit_to_hexnum(search_str_end_pos,search_str_pos,search_str_end_pos-search_str_pos);
                cPrintLog(CDBG_SIGFOX_DBG,"** search_data %s %d\n",search_str_pos,(search_str_end_pos-(search_str_pos+search_str_len)));

                for(i=0;i<(search_str_end_pos-(search_str_pos+search_str_len));i++){
                    if(search_str_pos[i+search_str_len] != '0'){
                        received_fail = true;
                        break;
                    }
                }
                if(received_fail){
                    cPrintLog(CDBG_SIGFOX_INFO, "stx_error False %s\n",search_str_pos);
                }
                else{
                    received_ok = true;
                    cPrintLog(CDBG_SIGFOX_INFO, "str_error OK %s\n",search_str_pos);
                }
            }
        }
        else
        {
            if (search_str_pos && !received_ok)
            {
                received_ok = 1;
                if (downlink_str_len)
                {
                    memset(downlink_data, 0x00, sizeof(downlink_data));
                    memcpy(downlink_data, search_str_pos + search_str_len, downlink_str_len);
                    cPrintLog(CDBG_SIGFOX_DBG, "** downlink_data:%s\n", downlink_data);
                }
                else
                {
                    cPrintLog(CDBG_SIGFOX_DBG, "** str_error OK\n");
                }
                downlink_data_size = downlink_str_len;
            }
        }
#else
        if(sfx_error_check){
            search_str_pos = (uint8_t *)strstr((const char *)received_data, (const char *)sfx_err_str);

            if (search_str_pos && !received_ok && !received_fail){
                // uint8_t *search_str_end_pos;
                uint8_t i,len;

                len = strlen((const char *)search_str_pos+sfx_err_str_len);
                for(i=0;i<len;i++){
                    if(!isxdigit(search_str_pos[i+sfx_err_str_len])){
                        cPrintLog(CDBG_SIGFOX_DBG, "str_error OK %s\n",search_str_pos);
                        received_ok = true;
                        break;
                    }
                    else if(search_str_pos[i+sfx_err_str_len] != '0'){
                        cPrintLog(CDBG_SIGFOX_ERR, "stx_error False %s\n",search_str_pos);
                        received_fail = true;
                        break;
                    }
                }
            }
        }
        if(sfx_return_check && !data_received){
            search_str_pos = (uint8_t *)strstr((const char *)received_data, (const char *)search_str);

            if (search_str_pos)
            {
                if (downlink_str_len)
                {
                    data_received = true;
                    memset(downlink_data, 0x00, sizeof(downlink_data));
                    memcpy(downlink_data, search_str_pos + search_str_len, downlink_str_len);
                    cPrintLog(CDBG_SIGFOX_DBG, "** downlink_data:%s\n", downlink_data);
                }
                else
                {
                    received_ok = true;
                    cPrintLog(CDBG_SIGFOX_INFO, "Downlink data OK\n");
                }
                downlink_data_size = downlink_str_len;
            }
        }
#endif
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[256];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (sizeof(data_array))))
            {
                if(!sigfox_is_bypass_mode())
                {
                    index = index - 2;
                    data_array[index] = '\0';
                }
                sigfox_received_data(data_array, index);
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            cPrintLog(CDBG_SIGFOX_ERR, "%s %d APP_UART_COMMUNICATION_ERROR!\n", __func__, __LINE__);
            break;

        case APP_UART_FIFO_ERROR:
            cPrintLog(CDBG_SIGFOX_ERR, "%s %d APP_UART_FIFO_ERROR!\n", __func__, __LINE__);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void cfg_sigfox_uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        PIN_DEF_SIGFOX_UART_RX, //RX_PIN_NUMBER,
        PIN_DEF_SIGFOX_UART_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

static void cfg_sigfox_uart_uninit(void)
{
    app_uart_close();
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_UART_RX);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_UART_RX, 0);
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_UART_TX);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_UART_TX, 0);
}

static void sigfox_state_handler(void * p_context)
{
    static uint32_t timeout;
    static sigfox_state_s sfx_send_type;

    switch(m_sigfox_state)
    {
        case SETUP_S:
            // cPrintLog(CDBG_SIGFOX_DBG,"** SETUP_S\n");
            if(!sigfox_power_on_old_status)
            {
                if(sigfox_power_on_step_settup_s == 0)
                {
                    sigfox_power_uart_to_low();
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s == 1)
                {
                    cfg_board_common_power_control(module_comm_pwr_sigfox, true);
                    nrf_delay_ms(4);
                    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
//                    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
                    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
//                    nrf_delay_ms(1);  //spec is 30us
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s == 2)
                {
//                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);  //spec is 1.82 ms
                    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s >= 3 && sigfox_power_on_step_settup_s <= (3+(SIGFOX_DELAY_FOR_UART_INIT/SIGFOX_STATE_MACHINE_TICK_MS)))
                {
                    //wait sigfox booting 1st for uart error
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s == ((3+(SIGFOX_DELAY_FOR_UART_INIT/SIGFOX_STATE_MACHINE_TICK_MS))+1))
                {
                    cfg_sigfox_uart_init();
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s <= ((3+(SIGFOX_DELAY_FOR_UART_INIT/SIGFOX_STATE_MACHINE_TICK_MS))+2+(SIGFOX_DELAY_FOR_READY/SIGFOX_STATE_MACHINE_TICK_MS)))
                {
                    //wait sigfox booting 2nd for cli command
                    sigfox_power_on_step_settup_s++;
                }
                else
                {
                    sigfox_power_on_step_settup_s = 0;
                    sigfox_power_on_old_status = true;
                }
            }
            else
            {
                memset(downlink_data, 0, sizeof(downlink_data));
                downlink_data_size = 0;
                if(m_get_parameter)
                    m_sigfox_state = GET_ID_S;
                else if(m_set_parameter)
                    m_sigfox_state = EXIT;
                else if(scan_rc_parameter)
                    m_sigfox_state = SCAN_RC_S;
                else
                    m_sigfox_state = INIT_S;
            }
            break;
        case INIT_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** INIT_S\n");

            if(m_downlink_msg_on){
                cPrintLog(CDBG_SIGFOX_INFO,"DownLink Enabled\n");
                sfx_send_type = TRANSMIT_FRAME_DOWNLINK_S;
            }
            else{
                sfx_send_type = TRANSMIT_FRAME_S;
            }
            m_sigfox_abort_flag = false;
            m_sigfox_state = INIT_R;
            break;
        case INIT_R:
            m_sigfox_state = OPEN_S;
            break;
        case OPEN_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** OPEN_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_OPEN_CMD;
            // timeout= CFG_UART_TIMEOUT;
            timeout= SFX_SETUP_TIMEOUT;
            sigfox_Send_Command();
            m_sigfox_state = OPEN_R;
            break;
        case OPEN_R:
            if(received_ok){
                m_sigfox_state = SET_STD_CFG_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s OPEN Error!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s OPEN Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SET_STD_CFG_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SET_STD_CFG_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            switch(module_parameter_get_val(module_parameter_item_sigfox_RC_number)){
                case 2:
                case 3:
                case 4:
                case 5:
                    m_sigfox_cmd = SIGFOX_SET_STD_CMD;
                    // timeout = CFG_UART_TIMEOUT;
                    timeout = SFX_SETUP_TIMEOUT;
                    sigfox_Send_Command();
                    m_sigfox_state = SET_STD_CFG_R;
                    break;
                default:
                    m_sigfox_state = SW_PUBLIC_S;
                    break;
            }
           break;
        case SET_STD_CFG_R:
            if (received_ok)
            {
                m_sigfox_state = SW_PUBLIC_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_STD_CFG_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if (!(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_STD_CFG_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SW_PUBLIC_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SW_PUBLIC_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            if(m_snek_testmode_enable){
                m_sigfox_cmd = SIGFOX_SET_PUBLIC_CMD;
                // timeout = CFG_UART_TIMEOUT;
                timeout = SFX_SETUP_TIMEOUT;
                sigfox_Send_Command();
                m_sigfox_state = SW_PUBLIC_R;
            }
            else{
                m_sigfox_state = SET_PAY_ENC_S;
            }
            break;
        case SW_PUBLIC_R:
            if (received_ok)
            {
                m_sigfox_state = SET_PAY_ENC_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SW_PUBLIC_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if (!(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_ERR, "%s SW_PUBLIC_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SET_PAY_ENC_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SET_PAY_ENC_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            if(PAYLOAD_ENCRIPTION_ENABLE){
                m_sigfox_cmd = SIGFOX_SET_ENCRIPTION_CMD;
                // timeout = CFG_UART_TIMEOUT;
                timeout = SFX_SETUP_TIMEOUT;
                sigfox_Send_Command();
                m_sigfox_state = SET_PAY_ENC_R;
            }
            else{
                m_sigfox_state = SW_PA_S;
            }
            break;
        case SET_PAY_ENC_R:
            if(received_ok){
                m_sigfox_state = SW_PA_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_PAY_ENC_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if (!(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_PAY_ENC_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SW_PA_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SW_PA_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            switch(module_parameter_get_val(module_parameter_item_sigfox_RC_number)){
                case 2:
                case 4:
                    m_sigfox_cmd = SIGFOX_SET_PA_CMD;
                    // timeout = CFG_UART_TIMEOUT;
                    timeout = SFX_SETUP_TIMEOUT;
                    sigfox_Send_Command();
                    m_sigfox_state = SW_PA_R;
                    break;
                default :
                    m_sigfox_state = SET_POWERLEVEL_S;
                    break;
            }
            break;
        case SW_PA_R:
            if(received_ok){
                m_sigfox_state = SET_POWERLEVEL_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SW_PA_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if (!(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SW_PA_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case TRANSMIT_FRAME_S:
            cPrintLog(CDBG_SIGFOX_INFO,"SendFrame Only\n");
            cPrintLog(CDBG_SIGFOX_DBG,"** TRANSMIT_FRAME_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            if(mSTOPmessage&&!mWifimsg_send){
                m_sigfox_cmd = SIGFOX_STOP_MESSAGE_CMD;
            }
            else{
                m_sigfox_cmd = SIGFOX_SF_CMD;
            }
            sigfox_Send_Command();
            timeout = SFX_TRANSMIT_TIMEOUT;
            m_sigfox_state = TRANSMIT_FRAME_R;
            break;
        case TRANSMIT_FRAME_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_FRAME_R!\n", __func__);

                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = false;
                m_sigfox_state = EXIT;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_FRAME_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_FRAME_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
            }
            break;
        case TRANSMIT_FRAME_DOWNLINK_S:
            cPrintLog(CDBG_SIGFOX_INFO,"SendFrame And Wiat DL\n");
            cPrintLog(CDBG_SIGFOX_DBG,"** TRANSMIT_FRAME_DOWNLINK_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_SF_R_CMD;
            sigfox_Send_Command();
            cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT AND WAIT!\n", __func__);
            timeout = SFX_TRANSMIT_DN_TIMEOUT;
            m_sigfox_state = TRANSMIT_FRAME_DOWNLINK_R;
            break;
        case TRANSMIT_FRAME_DOWNLINK_R:
            if(data_received)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_R!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = false;
                m_sigfox_state = SAVE_DOWNLINK_S;
                received_ok = 0;
                received_fail = 0;
            }
            else if(received_fail)
            {
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
                cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_Fail!\n", __func__);
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
                cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_TIMEOUT!\n", __func__);
            }
            break;
        case TRANSMIT_BIT_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** TRANSMIT_BIT_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_SB_CMD;
            sigfox_Send_Command();
            timeout = SFX_TRANSMIT_TIMEOUT;
            m_sigfox_state = TRANSMIT_BIT_R;
            break;
        case TRANSMIT_BIT_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_BIT_R!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = false;
                m_sigfox_state = EXIT;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_BIT_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_BIT_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
            }
            break;
        case TRANSMIT_BIT_DOWNLINK_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** TRANSMIT_BIT_DOWNLINK_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_SB_R_CMD;
            sigfox_Send_Command();
            cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT AND WAIT!\n", __func__);
            timeout = SFX_TRANSMIT_DN_TIMEOUT;
            m_sigfox_state = TRANSMIT_BIT_DOWNLINK_R;
            break;
        case TRANSMIT_BIT_DOWNLINK_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_R!\n", __func__);
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = false;
                m_sigfox_state = SAVE_DOWNLINK_S;
                received_ok = 0;
                received_fail = 0;
            }
            else if(received_fail)
            {
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
                cPrintLog(CDBG_SIGFOX_ERR, "%s RECEIVE_DOWNLINK_Fail!\n", __func__);
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                sigfox_transmit_fail = true;
                m_sigfox_state = EXIT;
                cPrintLog(CDBG_SIGFOX_ERR, "%s RECEIVE_DOWNLINK_TIMEOUT!\n", __func__);
            }
            break;
        case BYPASS_INIT:
#ifdef FEATURE_CFG_BYPASS_CONTROL
            if (m_sigfox_state_init_flag)
            {
                sigfox_power_on(true);
                m_sigfox_state_init_flag = false;
                m_sigfox_proc_counter = 0;
                m_sigfox_abort_flag = false;
            }
            else
            {
                if (++m_sigfox_proc_counter > SIGFOX_BOOT_WAIT_TICK_MONARCH) //wait boot time
                {
                    m_sigfox_state_init_flag = true;
                    m_sigfox_state = BYPASS_WORK;
                }
            }
#else
            m_sigfox_state_init_flag = true;
            m_sigfox_state = EXIT;
#endif
            break;
        case BYPASS_WORK:
#ifdef FEATURE_CFG_BYPASS_CONTROL
            if (m_sigfox_state_init_flag)
            {
                m_sigfox_state_init_flag = false;
                if (cb_cfg_sigfox_bypass_noti)
                    cb_cfg_sigfox_bypass_noti(true, "");
                cPrintLog(CDBG_SIGFOX_INFO, "%s %d start SIGFOX BYPASS_WORK ready!\n", __func__, __LINE__);
            }
            else
            {
                if (m_sigfox_abort_flag)
                {
                    m_sigfox_abort_flag = false;
                    m_sigfox_state_init_flag = true;
                    cb_cfg_sigfox_bypass_recv = NULL;
                    cb_cfg_sigfox_bypass_noti = NULL;
                    m_sigfox_state = EXIT;
                }
             }
#else
            m_sigfox_state_init_flag = true;
            m_sigfox_state = EXIT;
#endif
            break;
        case GET_ID_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** GET_ID_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_GET_ID_CMD;
            sigfox_Send_Command();
            timeout = SFX_SETUP_TIMEOUT;
            m_sigfox_state = GET_ID_R;
            break;
        case GET_ID_R:
            if(data_received)
            {
                m_sigfox_state_init_flag = true;
                sigfox_hexdigit_to_hexnum(m_module_peripheral_ID.sigfox_device_ID,downlink_data,8);
                m_sigfox_state = GET_PACCODE_S;
                received_ok = 0;
                received_fail = 0;
            }
            else if( !(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_ERR, "%s GET_ID_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case GET_PACCODE_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** GET_PACCODE_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_GET_PACCODE_CMD;
            sigfox_Send_Command();
            timeout = SFX_SETUP_TIMEOUT;
            m_sigfox_state = GET_PACCODE_R;
            break;
        case GET_PACCODE_R:
            if(data_received)
            {
                m_sigfox_state_init_flag = true;
                cPrintLog(CDBG_SIGFOX_DBG,"** PAC : %s\n",downlink_data);
                sigfox_hexdigit_to_hexnum(m_module_peripheral_ID.sigfox_pac_code,downlink_data,16);
                m_sigfox_state = EXIT;
//                m_sigfox_state = SET_RSSI_S;
                received_ok = 0;
                received_fail = 0;
            }
            else if( !(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_ERR, "%s GET_PACCODE_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SET_RSSI_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SET_RSSI_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_SET_RSSI_CMD;
            // timeout = CFG_UART_TIMEOUT;
            timeout = SFX_SETUP_TIMEOUT;
            sigfox_Send_Command();
            m_sigfox_state = SET_RSSI_R;
            break;
        case SET_RSSI_R:
            if(received_ok){
                m_sigfox_state_init_flag = true;
                m_sigfox_state = SET_LBT_S;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_RSSI_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_RSSI_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SET_LBT_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SET_LBT_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_cmd = SIGFOX_SET_LBT_CMD;
            // timeout = CFG_UART_TIMEOUT;
            timeout = SFX_SETUP_TIMEOUT;
            sigfox_Send_Command();
            m_sigfox_state = SET_LBT_R;
            break;
        case SET_LBT_R:
            if(received_ok){
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_LBT_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_LBT_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SET_POWERLEVEL_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SET_POWERLEVEL_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            if(m_powerlevel != 0){
                m_sigfox_cmd = SIGFOX_SET_POWERLEVEL_CMD;
                // timeout = CFG_UART_TIMEOUT;
                timeout = SFX_SETUP_TIMEOUT;
                sigfox_Send_Command();
                m_sigfox_state = SET_POWERLEVEL_R;
            }
            else{
                m_sigfox_state = sfx_send_type;
            }
            break;
        case SET_POWERLEVEL_R:
            if(received_ok){
                m_sigfox_state = sfx_send_type;
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_POWERLEVEL_R Fail!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout)){
                cPrintLog(CDBG_SIGFOX_ERR, "%s SET_POWERLEVEL_R Timeout!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SAVE_DOWNLINK_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SAVE_DOWNLINK_S\n");
            sigfox_get_ap_key(downlink_convert);
            cPrintLog(CDBG_SIGFOX_INFO, "%02X %02X %02X %02X %02X %02X %02X %02X RECEIVED\n", downlink_convert[0], downlink_convert[1], downlink_convert[2], downlink_convert[3], downlink_convert[4], downlink_convert[5], downlink_convert[6], downlink_convert[7]);
            m_sigfox_state = SAVE_DOWNLINK_R;
            // sigfox_downlink_success = true;
            break;
        case SAVE_DOWNLINK_R:
            if((m_module_parameter.downlink_version < downlink_convert[0]) || (downlink_convert[0] == 0xff))
            {
                module_parameter_set_val(module_parameter_item_idle_time, (unsigned int)(ONE_DAY_SEC/((unsigned int)downlink_convert[0])));
                module_parameter_set_val(module_parameter_item_no_motion_duration, (unsigned int)(downlink_convert[1]*60));

                downlink_max = (ONE_DAY_SEC / m_module_parameter.idle_time)*m_module_parameter.downlink_day;
                module_parameter_update();
            }
            m_sigfox_state = EXIT;
            break;
        case SCAN_RC_S:
            cPrintLog(CDBG_SIGFOX_DBG,"** SCAN_RC_S\n");
            received_ok = false;
            received_fail = false;
            data_received = false;
            m_sigfox_abort_flag = false;
            timeout = SFX_RC_SCAN_TIMEOUT;

            m_sigfox_cmd = SIGFOX_SCAN_RC_CMD;
            sigfox_Send_Command();
            m_sigfox_state = SCAN_RC_R;
            break;
        case SCAN_RC_R:
            if(data_received)
            {
#if 0       // Detected RC check 
                uint8_t rc_tmp;

                m_sigfox_state_init_flag = true;
                rc_tmp = hextonum(downlink_data[0]);
                if(rc_tmp){
                    m_module_parameter.sigfox_RC_number = rc_tmp;
                    cPrintLog(CDBG_SIGFOX_INFO,"SIGFOX RC scan data:%d\n",m_module_parameter.sigfox_RC_number);
                    sigfox_rc_checked  = true;
                }
                else
                    sigfox_rc_checked  = false;

                m_sigfox_state = EXIT;
#else
// TODO RC check
                uint8_t rc_tmp,cnv_data;

                if(isxdigit(downlink_data[1])){
                    // cnv_data = hextonum(downlink_data[0]) * 0x10 + hextonum(downlink_data[1]);
                    cnv_data = hextonum(downlink_data[0])*0x10;
                }
                else{
                    cnv_data = hextonum(downlink_data[0]);
                }
                switch(cnv_data){
                    case 0x1:
                        rc_tmp = 1;
                        break;
                    case 0x2:
                        rc_tmp = 2;
                        break;
                    case 0x4:
                        rc_tmp = 3;
                        break;
                    case 0x8:
                        rc_tmp = 4;
                        break;
                    case 0x10:
                        rc_tmp = 5;
                        break;
                    case 0x20:
                        rc_tmp = 6;
                        break;
                    default:
                        rc_tmp = 0;
                        break;
                }
                m_sigfox_state_init_flag = true;
                if(rc_tmp){
                    cPrintLog(CDBG_SIGFOX_INFO,"SIGFOX RC scan data:%d\n",rc_tmp);
                    sigfox_rc_checked  = true;
                    if(rc_tmp != m_module_parameter.sigfox_RC_number){
                        m_module_parameter.sigfox_RC_number = rc_tmp;
                        module_parameter_update();
                    }
                }
                else{
                    cPrintLog(CDBG_SIGFOX_INFO,"SIGFOX RC scan Fail\n");
                    sigfox_rc_checked  = false;
                }

                m_sigfox_state = EXIT;
#endif
            }
            else if(received_fail){
                cPrintLog(CDBG_SIGFOX_INFO,"SIGFOX RC scan fail\n");
                sigfox_rc_checked  = false;
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout))
            {
                cPrintLog(CDBG_SIGFOX_INFO,"SIGFOX RC scan timeout\n");
                sigfox_rc_checked  = false;
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case EXIT:
            cPrintLog(CDBG_SIGFOX_DBG,"** EXIT\n");
            if(sigfox_power_on_old_status)
                sigfox_power_on(false);
            if(m_sigfox_state_init_flag == true)
            {
                m_sigfox_abort_flag = false;
                m_sigfox_state_init_flag = false;
            }
            received_fail  = false;
            m_set_parameter = false;
            m_get_parameter = false;
            break;
        default:
            break;
    }
}

bool cfg_sigfox_downlink_on_off(bool on_off)
{
    bool old_downlink_msg_on;
    old_downlink_msg_on = m_downlink_msg_on;
    m_downlink_msg_on = on_off;
    return old_downlink_msg_on;
}

bool cfg_sigfox_set_senk_testmode_enable(bool enable)
{
    bool old_downlink_msg_on;
    // old_downlink_msg_on = m_snek_testmode_enable;
    m_snek_testmode_enable = enable;
    return old_downlink_msg_on;
}

static void cfg_sigfox_timer_create(void)
{
    uint32_t err_code;


    // Create timers.
    err_code = app_timer_create(&m_sigfox_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sigfox_state_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_sigfox_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_sigfox_timer_id, APP_TIMER_TICKS(SIGFOX_STATE_MACHINE_TICK_MS), NULL);
    APP_ERROR_CHECK(err_code);
}
void cfg_sigfox_timers_stop(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_stop(m_sigfox_timer_id);
    APP_ERROR_CHECK(err_code);
}

#endif
/**
 * @}
 */
