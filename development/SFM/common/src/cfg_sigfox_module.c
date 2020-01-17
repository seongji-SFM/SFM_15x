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

int m_data_module_type = 
#if defined(CDEV_SIGFOX_MODULE)
    DATA_MODULE_TYPE_SF_ONSEMI
#elif defined(CDEV_SIGFOX_MONARCH_MODULE)
    DATA_MODULE_TYPE_SF_MONARCH
#else
    DATA_MODULE_TYPE_NONE
#endif
;

#ifdef CDEV_SIGFOX_MODULE
#define SIGFOX_DUMMY_COMMAND                "AT\r\n"
#define SIGFOX_SF_COMMAND                        "AT$SF="
#define SIGFOX_OOB_COMMAND                     "AT$SO\r\n"
#define SIGFOX_POWERMODE_COMMAND       "AT$P=1\r\n"
#define SIGFOX_BREAK_COMMAND                   " "
#define SIGFOX_SNEK_COMMAND                   "ATS410=1\r\n"
#define SIGFOX_ID_COMMAND              "AT$I=10\r\n" 
#define SIGFOX_PACCODE_COMMAND         "AT$I=11\r\n"
#define SIGFOX_CHECK_CHANNEL_COMMAND   "AT$GI?\r\n"
#define SIGFOX_RESET_CHANNEL_COMMAND   "AT$RC\r\n"
#define SIGFOX_FREQUENCY_COMMAND      "AT$I=7\r\n"
#define SIGFOX_SET_POWERLEVEL_COMMAND "ATS302="
#define SIGFOX_SAVE_CONFIG_COMMAND "AT$WR\r\n"

sigfox_state_s m_sigfox_state;
sigfox_cmd_c m_sigfox_cmd;
bool m_sigfox_state_init_flag;
bool m_sigfox_abort_flag;
unsigned int m_sigfox_proc_counter;

uint8_t received_ok;
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define CFG_UART_TIMEOUT                50


APP_TIMER_DEF(m_sigfox_timer_id);                        /**SIGFOX timer. */

uint8_t frame_data[(SIGFOX_SEND_PAYLOAD_SIZE*2)+1];  //for hexadecimal
uint8_t downlink_data[40];
uint8_t downlink_data_size;
uint8_t downlink_convert[8];

volatile bool m_get_parameter = false;
volatile bool m_set_parameter = false;
bool m_downlink_msg_on = false;
bool m_snek_testmode_enable = false;

bool is_rcz24 = true;
int m_powerlevel = 0;
bool m_save_result = false;

extern uint32_t downlink_max;

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

bool cfg_sigfox_set_powerlevel(int level)   
{
    m_powerlevel = level;
    m_set_parameter = true;
//  cfg_sigfox_timer_create();
    sigfox_set_state(SETUP_S);
    cfg_sigfox_timers_start();
    while(m_set_parameter);
    cfg_sigfox_timers_stop();
    sigfox_set_state(SETUP_S);
    if(m_save_result)
    {
        m_save_result = false;
        return true;
    }
    else
    {
        return false;
    }
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
            nrf_delay_ms(100);
            //bus init
            cfg_sigfox_uart_init();
            nrf_delay_ms(10);
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

void sigfox_set_rcz(sigfox_rcz which_rcz)
{
    if((which_rcz ==  RCZ_2) || (which_rcz ==  RCZ_4))
    {
        is_rcz24 = true;
    }
    else
    {
        is_rcz24 = false;
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

uint32_t sigfox_bypass_put(uint8_t byte)
{
    if(m_sigfox_state == BYPASS_WORK)
    {
        return app_uart_put(byte);
    }
    return NRF_ERROR_INVALID_STATE;
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
        case SIGFOX_DUMMY_CMD : 
            sprintf((char*)send, SIGFOX_DUMMY_COMMAND) ; 
            break ; 
        case SIGFOX_SF_CMD : 
            sprintf((char*)send,"%s%s\r\n", SIGFOX_SF_COMMAND,frame_data) ; 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send request %s", send);
            break ;  
        case SIGFOX_SF_R_CMD :
            sprintf((char*)send,"%s%s,1\r\n", SIGFOX_SF_COMMAND,frame_data) ; 
            cPrintLog(CDBG_SIGFOX_INFO, "sigfox send request %s", send);        
            break;
        case SIGFOX_OOB_CMD:
            break;
        case SIGFOX_POWERMODE_CMD : 
            sprintf((char*)send, SIGFOX_POWERMODE_COMMAND) ; 
            break ; 
        case SIGFOX_BREAK_CMD:
            sprintf((char*)send, SIGFOX_BREAK_COMMAND);
            break;
        case SIGFOX_SNEK_CMD:
            sprintf((char*)send, SIGFOX_SNEK_COMMAND);
            break;   
        case SIGFOX_ID_CMD:
            sprintf((char*)send, SIGFOX_ID_COMMAND);
            break;
        case SIGFOX_PACCODE_CMD:
            sprintf((char*)send, SIGFOX_PACCODE_COMMAND);
            break;
        case SIGFOX_CHECK_CHANNEL_CMD:
            sprintf((char*)send, SIGFOX_CHECK_CHANNEL_COMMAND);
            break;
        case SIGFOX_RESET_CHANNEL_CMD:
            sprintf((char*)send, SIGFOX_RESET_CHANNEL_COMMAND);
            break;
        case SIGFOX_FREQUENCY_CMD:
            sprintf((char*)send, SIGFOX_FREQUENCY_COMMAND);
            break;
        case SIGFOX_SET_POWERLEVEL_CMD:
            sprintf((char*)send,"%s%d\r\n", SIGFOX_SET_POWERLEVEL_COMMAND,m_powerlevel);
            break;
        case SIGFOX_SAVE_CONFIG_CMD:
            sprintf((char*)send, SIGFOX_SAVE_CONFIG_COMMAND);
            break;

    }
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
    int s_len,k_len;
    for(s_len = 3, k_len=0; s_len < downlink_data_size;s_len++)
    {
        if(downlink_data[s_len]!=' ')
        {
            temp[k_len++]=downlink_data[s_len];
        }
    }
    sigfox_hexdigit_to_hexnum(received_data, temp, k_len);
}

void sigfox_received_data(uint8_t * received_data, uint8_t length)
{
    cPrintLog(CDBG_SIGFOX_DBG, "%s %d Uart Data Recv! length:%d\n", __func__, __LINE__, length);
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
        memset(downlink_data,0x00,sizeof(downlink_data));
        memcpy(downlink_data,received_data,length);
        downlink_data_size = length;
        if ( strncmp((char *)received_data, (char*)"OK",2) == 0 )
        {
            switch( m_sigfox_cmd ) 
            {
                case SIGFOX_DUMMY_CMD : 
                case SIGFOX_SF_CMD : 
                case SIGFOX_POWERMODE_CMD :
                case SIGFOX_SNEK_CMD:
                case SIGFOX_RESET_CHANNEL_CMD:
                case SIGFOX_SET_POWERLEVEL_CMD:
                case SIGFOX_SAVE_CONFIG_CMD:
                    received_ok = 1;
                    break ;
                default:
                    break;
            }
        }
        else if(strncmp((char *)received_data, (char*)"RX",2) == 0 )
        {
             received_ok = 1;
        }
        
        
        else
        {
            switch( m_sigfox_cmd ) 
            {
                case SIGFOX_CHECK_CHANNEL_CMD:
                case SIGFOX_ID_CMD :
                case SIGFOX_PACCODE_CMD :
                case SIGFOX_FREQUENCY_CMD:
                    received_ok = true;
                    break ;  
                default:
                    break;
            }
        }
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
    static uint8_t data_array[64];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (64)))
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
        UART_BAUDRATE_BAUDRATE_Baud9600
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
    switch(m_sigfox_state)
    {
        case SETUP_S:
#if 0
            memset(downlink_data, 0, sizeof(downlink_data));
            downlink_data_size = 0;

            sigfox_power_on(false);
            nrf_delay_ms(20);

            sigfox_power_on(true);
            nrf_delay_ms(1000);

            if(m_get_parameter)
            m_sigfox_state = GET_ID_S;
            else if(m_set_parameter)
            m_sigfox_state = SET_POWERLEVEL_S;
            else
            m_sigfox_state = INIT_S;
#else  //power on delay tuning
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
                    nrf_delay_ms(2);
                    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
                    nrf_delay_ms(5);  //spec is 4ms
//                    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
                    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s == 2)
                {
//                    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
                    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s == 3)
                {
                    cfg_sigfox_uart_init();
                    sigfox_power_on_step_settup_s++;
                }
                else if(sigfox_power_on_step_settup_s <= 6)
                {
                    //wait sigfox booting
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
                    m_sigfox_state = SET_POWERLEVEL_S;
                else
                    m_sigfox_state = INIT_S;
            }
#endif
            break;
        case INIT_S:
            received_ok = 0;
            m_sigfox_abort_flag = false;
            if(m_snek_testmode_enable)
            {
                m_sigfox_cmd = SIGFOX_SNEK_CMD;
            }
            else
            {
                m_sigfox_cmd = SIGFOX_DUMMY_CMD;
            }
            timeout= CFG_UART_TIMEOUT;
            sigfox_Send_Command();
            m_sigfox_state = INIT_R;
            break;
        case INIT_R:
            if(received_ok)
            {
                if(is_rcz24)
                {
                    m_sigfox_state = CHECK_CHANNEL_S;
                }
                else
                {
                    if(m_downlink_msg_on)
                    {
                        m_sigfox_state = TRANSMIT_FRAME_DOWNLINK_S;
                    }
                    else
                    {
                        m_sigfox_state = TRANSMIT_FRAME_S;
                    }
                }
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }

            break;
        case CHECK_CHANNEL_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_CHECK_CHANNEL_CMD;
            sigfox_Send_Command();
            timeout = CFG_UART_TIMEOUT;
            m_sigfox_state = CHECK_CHANNEL_R;
            break;
        case CHECK_CHANNEL_R:
            if(received_ok)
            {
                m_sigfox_state_init_flag = true;
                if(cfg_sigfox_check_channel(downlink_data))
                {
                    m_sigfox_state = RESET_CHANNEL_S;
                }
                else
                {
                    if(m_downlink_msg_on)
                    {
                        m_sigfox_state = TRANSMIT_FRAME_DOWNLINK_S;
                    }
                    else
                    {
                        m_sigfox_state = TRANSMIT_FRAME_S;
                    }
                }
                received_ok = 0;
            }

            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case RESET_CHANNEL_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_RESET_CHANNEL_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = RESET_CHANNEL_R;
            break;
        case RESET_CHANNEL_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s RESET_CHANNEL_R!\n", __func__);
                nrf_delay_ms(30);
                if(m_downlink_msg_on)
                {
                    m_sigfox_state = TRANSMIT_FRAME_DOWNLINK_S;
                }
                else
                {
                    m_sigfox_state = TRANSMIT_FRAME_S;
                }
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case GET_ID_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_ID_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = GET_ID_R;
            break;
        case GET_ID_R:
            if(received_ok)
            {
                m_sigfox_state_init_flag = true;
                sigfox_hexdigit_to_hexnum(m_module_peripheral_ID.sigfox_device_ID,downlink_data,8);
                m_sigfox_state = GET_PACCODE_S;
                received_ok = 0;
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case GET_PACCODE_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_PACCODE_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = GET_PACCODE_R;
            break;
        case GET_PACCODE_R:
            if(received_ok)
            {
                m_sigfox_state_init_flag = true;
                sigfox_hexdigit_to_hexnum(m_module_peripheral_ID.sigfox_pac_code,downlink_data,16);
                m_sigfox_state = GET_FREQUENCY_S;
                received_ok = 0;
            }

            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case GET_FREQUENCY_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_FREQUENCY_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = GET_FREQUENCY_R;
            break;
        case GET_FREQUENCY_R:
            if(received_ok)
            {
                m_sigfox_state_init_flag = true;
                cPrintLog(CDBG_SIGFOX_INFO, "BAND : %s\n", downlink_data);
                if ( (strncmp((char *)downlink_data, (char*)"FCC",3) == 0)
                     || (strncmp((char *)downlink_data, (char*)"RC2",3) == 0)
                     || (strncmp((char *)downlink_data, (char*)"RC4",3) == 0)
			        )
                {
                    is_rcz24 = true;
                }
                else 
                {
                    is_rcz24 = false;
                }
                m_sigfox_state = EXIT;
                received_ok = 0;
            }

            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }

            break;
        case TRANSMIT_FRAME_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_SF_CMD;
            sigfox_Send_Command();
            timeout = 300;
            m_sigfox_state = TRANSMIT_FRAME_R;
            break;
        case TRANSMIT_FRAME_R:
            if(received_ok)
            {
//                m_sigfox_state = IDLE_S;
                cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT_FRAME_R!\n", __func__);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout))
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case TRANSMIT_FRAME_DOWNLINK_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_SF_R_CMD;
            sigfox_Send_Command();
            cPrintLog(CDBG_SIGFOX_INFO, "%s TRANSMIT AND WAIT!\n", __func__);
            timeout = 300;
            m_sigfox_state = TRANSMIT_FRAME_DOWNLINK_R;
            break;
        case TRANSMIT_FRAME_DOWNLINK_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_R!\n", __func__);
                m_sigfox_state_init_flag = true;
                //                m_sigfox_state = EXIT;
                m_sigfox_state = SAVE_DOWNLINK_S;
                received_ok = 0;
            }

            else if( !(--timeout))
            {
            m_sigfox_state_init_flag = true;
            m_sigfox_state = EXIT;
            cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVE_DOWNLINK_TIMEOUT!\n", __func__);
            }
            break;
        case SAVE_DOWNLINK_S:
            sigfox_get_ap_key(downlink_convert);
            cPrintLog(CDBG_SIGFOX_INFO, "%s RECEIVED\n", downlink_convert);
            m_sigfox_state = SAVE_DOWNLINK_R;
            break;
        case SAVE_DOWNLINK_R:
            if((m_module_parameter.downlink_version < downlink_convert[0]) || (downlink_convert[0] == 0xff))
            {
                if(downlink_convert[0] == 0xff)
                module_parameter_set_val(module_parameter_item_downlink_version, (unsigned int)0x00);
                else
                module_parameter_set_val(module_parameter_item_downlink_version, (unsigned int)downlink_convert[0]);
                module_parameter_set_val(module_parameter_item_idle_time, (unsigned int)(CFG_ONE_DAY_SEC/((unsigned int)downlink_convert[1])));
                module_parameter_set_val(module_parameter_item_downlink_day, (unsigned int)downlink_convert[2]);
                module_parameter_set_val(module_parameter_item_wifi_enable, (unsigned int)downlink_convert[3]);
                module_parameter_set_val(module_parameter_item_optional_mode, (unsigned int)downlink_convert[4]);
                downlink_max = (CFG_ONE_DAY_SEC / m_module_parameter.idle_time)*m_module_parameter.downlink_day;
                module_parameter_update();
            }
            m_sigfox_state = EXIT;
            break;
        case IDLE_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_POWERMODE_CMD;
            sigfox_Send_Command();
            m_sigfox_state = IDLE_R;
            break;
        case IDLE_R:
            if(received_ok)
            {
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case BYPASS_INIT:
#ifdef FEATURE_CFG_BYPASS_CONTROL
            if(m_sigfox_state_init_flag)
            {
                sigfox_power_on(true);
                m_sigfox_state_init_flag = false;
                m_sigfox_proc_counter = 0;
                m_sigfox_abort_flag = false;
            }
            else
            {
                if(++m_sigfox_proc_counter > SIGFOX_BOOT_WAIT_TICK)  //wait boot time
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
            if(m_sigfox_state_init_flag)
            {
                m_sigfox_state_init_flag = false;
                if(cb_cfg_sigfox_bypass_noti)cb_cfg_sigfox_bypass_noti(true, "");
                cPrintLog(CDBG_SIGFOX_INFO, "%s %d start SIGFOX BYPASS_WORK ready!\n", __func__, __LINE__);
            }
            else
            {
                if(m_sigfox_abort_flag)
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

        case SET_POWERLEVEL_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_SET_POWERLEVEL_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = SET_POWERLEVEL_R;
            break;
        case SET_POWERLEVEL_R:
            if(received_ok)
            {
                cPrintLog(CDBG_SIGFOX_INFO, "SET POWER LEVEL!\n");
                m_sigfox_state = SAVE_CONFIG_S;
            }
            else if( !(--timeout))
            {
                m_save_result = false;
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;
        case SAVE_CONFIG_S:
            received_ok = 0;
            m_sigfox_cmd = SIGFOX_SAVE_CONFIG_CMD;
            sigfox_Send_Command();
            timeout = 100;
            m_sigfox_state = SAVE_CONFIG_R;
            break;
        case SAVE_CONFIG_R:
            if(received_ok)
            {
                m_save_result = true;
                cPrintLog(CDBG_SIGFOX_INFO, "SAVE POWER LEVEL %d!\n",m_powerlevel);
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            else if( !(--timeout))
            {
                m_save_result = false;
                m_sigfox_state_init_flag = true;
                m_sigfox_state = EXIT;
            }
            break;

        case EXIT:
            sigfox_power_on(false);
            if(m_sigfox_state_init_flag == true)
            {
                received_ok = 0;
                m_sigfox_abort_flag = false;
                m_sigfox_state_init_flag = false;
            }
            m_set_parameter = false;
            m_get_parameter = false;
                break;
        default:
            break;
    }
}

uint8_t *cfg_sigfox_get_downlink_ptr(uint32_t *p_get_size)
{
    if(p_get_size)*p_get_size=downlink_data_size;
    return downlink_data;
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
    old_downlink_msg_on = m_snek_testmode_enable;
    m_snek_testmode_enable = enable;
    return old_downlink_msg_on;
}

void cfg_sigfox_timer_create()
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
    err_code = app_timer_start(m_sigfox_timer_id, APP_TIMER_TICKS(200), NULL);
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
