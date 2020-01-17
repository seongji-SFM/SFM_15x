/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
*/

#include <stdint.h>
#include <string.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_sdh.h"
#include "nrf_power.h"

#include "SEGGER_RTT.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_config.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_sigfox_module.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_ble_ctrl.h"
#include "cfg_special_boot_mode.h"
#include "cfg_full_test.h"

extern const nrf_drv_spi_config_t m_spi_config_default;

static bool m_cfg_bridge_from_uart_to_uart_flag = false;  //for sigfox uart bridge
static uint32_t m_cfg_testmode_wait_tick;
static uint8_t *m_cfg_board_testmode_tx_buf_uart;
static uint32_t m_cfg_board_testmode_tx_buf_uart_idx;
static uint8_t *m_cfg_board_testmode_rx_buf_uart;
static uint32_t m_cfg_board_testmode_rx_buf_uart_idx;
static uint8_t *m_cfg_board_testmode_rx_buf_rtt;  //down data to rtt
static uint32_t m_cfg_board_testmode_rx_buf_rtt_idx;  //down data to rtt
static uint8_t *m_cfg_board_testmode_tx_buf_rtt;  //up data to rtt
static uint32_t m_cfg_board_testmode_tx_buf_rtt_idx;  //up data to rtt
static uint8_t *m_cfg_board_testmode_tx_buf_uart_2nd;      //for sigfox uart bridge or NUS
static uint32_t m_cfg_board_testmode_tx_buf_uart_idx_2nd;  //for sigfox uart bridge or NUS
static uint8_t *m_cfg_board_testmode_rx_buf_uart_2nd;      //for sigfox uart bridge or NUS
static uint32_t m_cfg_board_testmode_rx_buf_uart_idx_2nd;  //for sigfox uart bridge or NUS

extern int dtm_mode(void);
static void cfg_board_RTT_read_N_clearCmd_proc(void);

static void cfg_board_testmode_wifi_download(void)
{
    bool led_on_off = false;
    int tickCnt = 0;
    int loopCnt = 0;

    cPrintLog(CDBG_FCTRL_INFO, "====Enter Download Mode====\n");

#if defined(CDEV_WIFI_MODULE)
    cWifi_enter_download_mode();
#endif

#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) && (CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB) && defined(CDEV_GPS_MODULE)
    nrf_delay_ms(20);
    cGps_gpio_init();
    cGps_download_power_control();  // for gps test mode
#endif

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
    nrf_delay_ms(20);
    sigfox_power_on(true);
#endif

    while(1)
    {
        cPrintLog(CDBG_FCTRL_INFO, "wifi download mode! %d\n", tickCnt++);
        (void)tickCnt;
        cfg_ble_led_control(led_on_off);
        led_on_off = !led_on_off;
        
        loopCnt = 200;
        while(--loopCnt)
        {

            cfg_board_RTT_read_N_clearCmd_proc();
            nrf_delay_ms(10);
        }
    }
}

static void cfg_board_RTT_read_N_clearCmd_proc(void)
{
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN

    rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
    if(rtt_rd_size == 2 && ((rtt_rd_bufffer[0] == 'C') && ((rtt_rd_bufffer[1] == 'R') || (rtt_rd_bufffer[1] == 'F'))))
    {
        if((rtt_rd_bufffer[1] == 'R'))
        {               
            cfg_board_reset();  //reset work
        }
        else if(rtt_rd_bufffer[1] == 'F')
        {
            cfg_nvm_factory_reset(true);  //factory reset work
        }
    }

}

static void cfg_board_RTT_reset_N_factory_reset_proc(void)
{

#ifdef FEATURE_CFG_RTT_MODULE_CONTROL
    SEGGER_RTT_printf(0, "\n==TBC_OVER_RTT Testmode==\n");  //start marker is "==TBC_OVER_RTT Testmode==" don't change this
#endif
    while(1)
    {
        cfg_board_RTT_read_N_clearCmd_proc();
    }
}

#ifdef CDEV_WIFI_MODULE
static void cfg_board_testmode_wifi(void)
{
    SEGGER_RTT_printf(0, "====enter wifi rf test mode====\n");
#ifdef CDEV_WIFI_MODULE
    cWifi_enter_rftest_mode();
#endif
    cfg_board_RTT_reset_N_factory_reset_proc();
}
#endif

#ifdef CDEV_WIFI_MODULE
static void cfg_board_testmode_wifi_always_on(void)
{
    SEGGER_RTT_printf(0, "====enter wifi always on mode====\n");
#ifdef CDEV_WIFI_MODULE
    cfg_board_gpio_set_default();
    cWifi_power_control(true);
#endif
    cfg_board_RTT_reset_N_factory_reset_proc();
}
#endif

#ifdef CDEV_GPS_MODULE
static const nrf_drv_spi_t m_cfg_board_testmode_gps_Spi = NRF_DRV_SPI_INSTANCE(GPS_SPI_INSTANCE);  /**< SPI instance. */
static volatile bool m_cfg_board_testmode_gps_spi_XferDone; 
static uint8_t *m_cfg_board_testmode_gps_spi_tx_buf;
static uint8_t *m_cfg_board_testmode_gps_spi_rx_buf;
static void cfg_board_testmode_gps_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_tx_buf_uart_idx < 128)
            {
                m_cfg_board_testmode_tx_buf_uart[m_cfg_board_testmode_tx_buf_uart_idx++] = uart_byte;
            }
//            app_uart_put(uart_byte);  //echo test
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_testmode_gps_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    (void)p_event;
    (void)p_context;
    m_cfg_board_testmode_gps_spi_XferDone = true;
}

static void cfg_board_testmode_gps(void)
{
//    cfg_ble_led_control(1);  //test power led

    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud9600
    };
    nrf_drv_spi_config_t spi_config;
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN
    int i;
    uint32_t tick_cnt = 0, tick_cnt_old_rtt = 0;

    SEGGER_RTT_printf(0, "====enter gps test mode====\n");
    cGps_gpio_init();
    cGps_power_control(1, 1);  //gps power on

    //Since the TBC can not be executed, it uses TBC buffers.
    if((1024 <= CTBC_TX_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        app_uart_buffers_t buffers;

        memcpy(&spi_config, &m_spi_config_default, sizeof(nrf_drv_spi_config_t));
        spi_config.ss_pin   = PIN_DEF_GPS_SPI_CS;
        spi_config.miso_pin = PIN_DEF_GPS_SPI_MISO;
        spi_config.mosi_pin = PIN_DEF_GPS_SPI_MOSI;
        spi_config.sck_pin  = PIN_DEF_GPS_SPI_SCK;
        spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
        spi_config.mode = NRF_DRV_SPI_MODE_0;
        spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
        APP_ERROR_CHECK(nrf_drv_spi_init(&m_cfg_board_testmode_gps_Spi, &spi_config, cfg_board_testmode_gps_spi_event_handler, NULL));
   
        m_cfg_board_testmode_gps_spi_tx_buf = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_gps_spi_rx_buf = &m_cTBC_tx_buf[128];
        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[256];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[384];

        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_tx_buf[512];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_tx_buf[640];
        buffers.tx_buf_size = 128;
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_testmode_gps_uart_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);

        m_cfg_board_testmode_rx_buf_rtt = &m_cTBC_tx_buf[768];
        m_cfg_board_testmode_tx_buf_rtt = &m_cTBC_tx_buf[896];

        while(1)
        {
            tick_cnt++;
            memset(m_cfg_board_testmode_gps_spi_tx_buf, 0xff, 128);
            if(m_cfg_board_testmode_tx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_gps_spi_tx_buf, m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_tx_buf_uart_idx);
                m_cfg_board_testmode_tx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
            }
            else
            {
                rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
                if(rtt_rd_size > 0)
                {
                    tick_cnt_old_rtt = tick_cnt;
                    for(i=0;i<rtt_rd_size;i++)
                    {
                        m_cfg_board_testmode_rx_buf_rtt[m_cfg_board_testmode_rx_buf_rtt_idx++] = rtt_rd_bufffer[i];
                    }

                }
                if((m_cfg_board_testmode_rx_buf_rtt_idx > 0) && ((tick_cnt_old_rtt + 2) == tick_cnt))
                {
                    memcpy(m_cfg_board_testmode_gps_spi_tx_buf, m_cfg_board_testmode_rx_buf_rtt, m_cfg_board_testmode_rx_buf_rtt_idx);
                    m_cfg_board_testmode_rx_buf_rtt_idx = 0;
                }
            }

            if(((m_cfg_board_testmode_gps_spi_tx_buf[0] == 'C') && ((m_cfg_board_testmode_gps_spi_tx_buf[1] == 'R') || (m_cfg_board_testmode_gps_spi_tx_buf[1] == 'F'))))
            {
                if((m_cfg_board_testmode_gps_spi_tx_buf[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(m_cfg_board_testmode_gps_spi_tx_buf[1] == 'F')
                {
                    cfg_nvm_factory_reset(true);  //factory reset work
                }
            }
            else
            {
                m_cfg_board_testmode_gps_spi_XferDone = false;
                nrf_drv_spi_transfer(&m_cfg_board_testmode_gps_Spi, m_cfg_board_testmode_gps_spi_tx_buf, 128, m_cfg_board_testmode_gps_spi_rx_buf, 128);
                while(!m_cfg_board_testmode_gps_spi_XferDone);

                for(i=0; i<128; i++)
                {
                    if(m_cfg_board_testmode_gps_spi_rx_buf[i] != 0xff)
                    {
                        m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = m_cfg_board_testmode_gps_spi_rx_buf[i];
                        m_cfg_board_testmode_tx_buf_rtt[m_cfg_board_testmode_tx_buf_rtt_idx++] = m_cfg_board_testmode_gps_spi_rx_buf[i];
                    }
                }

                if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
                {
                    for(i=0; i<m_cfg_board_testmode_rx_buf_uart_idx; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_rx_buf_uart[i]);
                    }
                    m_cfg_board_testmode_rx_buf_uart_idx = 0;
                }
                

                if(m_cfg_board_testmode_tx_buf_rtt_idx > 0)
                {
                    SEGGER_RTT_Write(0, m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_tx_buf_rtt_idx);
                    m_cfg_board_testmode_tx_buf_rtt_idx = 0;
                }
            }
            nrf_delay_ms(200);
        }
    }
}
#endif

static void cfg_board_bridge_from_RTT_to_uart_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx < 512)
            {
                m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = uart_byte;
            }
//            app_uart_put(uart_byte);  //echo test
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "GPS test uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_bridge_from_RTT_to_uart(uint32_t baud_rate, uint32_t rx_pin_no, uint32_t tx_pin_no)
{
    uint32_t                     err_code;
    app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN
    int i;
    uint32_t rtt_read_time_out=0;

    SEGGER_RTT_printf(0, "bridge_from_RTT_to_uart baud rate:%08x, rx:%d, tx:%d\n", baud_rate, rx_pin_no, tx_pin_no);
    comm_params.baud_rate = baud_rate;
    comm_params.rx_pin_no = rx_pin_no;
    comm_params.tx_pin_no = tx_pin_no;

    //Since the TBC can not be executed, it uses TBC buffers.
    if((2048 <= CTBC_TX_BUF_SIZE) && (256 <= CTBC_BYPASS_CMD_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        extern uint8_t m_cTBC_bypasscmd_buf[CTBC_BYPASS_CMD_BUF_SIZE];
        app_uart_buffers_t buffers;

        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[512];

        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_bypasscmd_buf[0];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_bypasscmd_buf[128];
        buffers.tx_buf_size = 128;
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_bridge_from_RTT_to_uart_uart_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);

        m_cfg_board_testmode_rx_buf_rtt = &m_cTBC_tx_buf[1024];
        m_cfg_board_testmode_tx_buf_rtt = &m_cTBC_tx_buf[1536];

        while(1)
        {
            rtt_read_time_out++;
            if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                m_cfg_board_testmode_tx_buf_rtt_idx = m_cfg_board_testmode_rx_buf_uart_idx;
                m_cfg_board_testmode_rx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
            }
            else
            {
                rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN
                if(rtt_rd_size > 0)
                {
                    for(i=0;i<rtt_rd_size;i++)
                    {
                        if(m_cfg_board_testmode_rx_buf_rtt_idx < 512)m_cfg_board_testmode_rx_buf_rtt[m_cfg_board_testmode_rx_buf_rtt_idx++] = rtt_rd_bufffer[i];
                    }
                    rtt_read_time_out = 0;
                }
                if((m_cfg_board_testmode_rx_buf_rtt_idx > 0) && rtt_read_time_out > 40000)  //about 100ms
                {
                    memcpy(m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_rx_buf_rtt, m_cfg_board_testmode_rx_buf_rtt_idx);
                    m_cfg_board_testmode_tx_buf_uart_idx = m_cfg_board_testmode_rx_buf_rtt_idx;
                    m_cfg_board_testmode_rx_buf_rtt_idx = 0;
                }
            }

            if((m_cfg_board_testmode_tx_buf_uart_idx == 2)
                && ((m_cfg_board_testmode_tx_buf_uart[0] == 'C') && ((m_cfg_board_testmode_tx_buf_uart[1] == 'R') || (m_cfg_board_testmode_tx_buf_uart[1] == 'F'))))
            {
                if((m_cfg_board_testmode_tx_buf_uart[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(m_cfg_board_testmode_tx_buf_uart[1] == 'F')
                {
                    cfg_nvm_factory_reset(true);  //factory reset work
                }
            }

            if(m_cfg_board_testmode_tx_buf_rtt_idx > 0)
            {
                SEGGER_RTT_Write(0, m_cfg_board_testmode_tx_buf_rtt, m_cfg_board_testmode_tx_buf_rtt_idx);
                m_cfg_board_testmode_tx_buf_rtt_idx = 0;
            }

            if(m_cfg_board_testmode_tx_buf_uart_idx > 0)
            {
                for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx; i++)
                {
                    app_uart_put(m_cfg_board_testmode_tx_buf_uart[i]);
                }
                m_cfg_board_testmode_tx_buf_uart_idx = 0;
            }
        }
    }
}

static void cfg_board_uart_bridge_from_external_to_sigfox_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx < 256)
            {
                m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = uart_byte;
                if(m_cfg_board_testmode_rx_buf_uart_idx == 2 && m_cfg_board_testmode_rx_buf_uart[0]=='C')
                {
                    if(m_cfg_board_testmode_rx_buf_uart[1]=='R')
                    {
                        cfg_board_reset();  //reset work
                    }
                    else if(m_cfg_board_testmode_rx_buf_uart[1]=='F')
                    {
                        cfg_nvm_factory_reset(true);  //factory reset work
                    }
                }

                if(uart_byte == '\n')
                {
                    m_cfg_bridge_from_uart_to_uart_flag = true;
                }
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "external uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "external uart fifo Err!\n");
            break;

        default:
            break;
    }
}

static void cfg_board_uart_bridge_from_sigfox_to_external_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            if(m_cfg_board_testmode_rx_buf_uart_idx_2nd < 256)
            {
                m_cfg_board_testmode_rx_buf_uart_2nd[m_cfg_board_testmode_rx_buf_uart_idx_2nd++] = uart_byte;
                if(uart_byte == '\n')
                {
                    m_cfg_testmode_wait_tick = 1;
                }
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "sigfox uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
            cPrintLog(CDBG_FCTRL_INFO, "sigfox uart fifo Err!\n");
            break;

        default:
            break;
    }
}


static void cfg_board_sigfox_bridge_from_uart(uint32_t baud_rate, uint32_t rx_pin_no, uint32_t tx_pin_no)
{
    uint32_t err_code;
    int i;
    bool bridge_from_uart_to_uart_flag_old;
    app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    app_uart_buffers_t buffers;

    app_uart_comm_params_t comm_params_sigfox =
    {
        PIN_DEF_SIGFOX_UART_RX, //RX_PIN_NUMBER,
        PIN_DEF_SIGFOX_UART_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud9600
    };
    app_uart_buffers_t buffers_sigfox;

    
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN

    SEGGER_RTT_printf(0, "sigfox_bridge_from_uart baud rate:%08x, in rx:%d, in tx:%d\n", baud_rate, rx_pin_no, tx_pin_no);
    comm_params.baud_rate = baud_rate;
    comm_params.rx_pin_no = rx_pin_no;
    comm_params.tx_pin_no = tx_pin_no;

    //Since the TBC can not be executed, it uses TBC buffers.
    if((2048 <= CTBC_TX_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        extern uint8_t m_cTBC_bypasscmd_buf[CTBC_BYPASS_CMD_BUF_SIZE];

        m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[0];
        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[256];
        m_cfg_board_testmode_tx_buf_uart_2nd = &m_cTBC_tx_buf[512];
        m_cfg_board_testmode_rx_buf_uart_2nd = &m_cTBC_tx_buf[768];


        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_tx_buf[1024];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_tx_buf[1152];
        buffers.tx_buf_size = 128;
        buffers_sigfox.rx_buf      = &m_cTBC_tx_buf[1280];
        buffers_sigfox.rx_buf_size = 128;
        buffers_sigfox.tx_buf      = &m_cTBC_tx_buf[1408];
        buffers_sigfox.tx_buf_size = 128;

        bridge_from_uart_to_uart_flag_old = m_cfg_bridge_from_uart_to_uart_flag = false;
        //connect external
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_uart_bridge_from_external_to_sigfox_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);
        

        while(1)
        {
            //ref cfg_board_RTT_reset_N_factory_reset_proc for RTT Factroy reset
            rtt_rd_size = SEGGER_RTT_Read(0, rtt_rd_bufffer, 64);  //check BUFFER_SIZE_DOWN 
            if(rtt_rd_size == 2 && ((rtt_rd_bufffer[0] == 'C') && ((rtt_rd_bufffer[1] == 'R') || (rtt_rd_bufffer[1] == 'F'))))
            {
                if((rtt_rd_bufffer[1] == 'R'))
                {               
                    cfg_board_reset();  //reset work
                }
                else if(rtt_rd_bufffer[1] == 'F')
                {
                    cfg_nvm_factory_reset(true);  //factory reset work
                }
            }

            if(bridge_from_uart_to_uart_flag_old != m_cfg_bridge_from_uart_to_uart_flag)
            {
                app_uart_close();
                if(m_cfg_bridge_from_uart_to_uart_flag)
                {
                    //connect sigfox
                    err_code = app_uart_init(&comm_params_sigfox, &buffers_sigfox, cfg_board_uart_bridge_from_sigfox_to_external_event_handle, APP_IRQ_PRIORITY_LOW);
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    //connect external
                    err_code = app_uart_init(&comm_params, &buffers, cfg_board_uart_bridge_from_external_to_sigfox_event_handle, APP_IRQ_PRIORITY_LOW);
                    APP_ERROR_CHECK(err_code);
                }
                nrf_delay_ms(1);
                if(m_cfg_bridge_from_uart_to_uart_flag)  //external to sigfox (uart connected to sigfox)
                {
                    CRITICAL_REGION_ENTER();
                    memcpy(m_cfg_board_testmode_tx_buf_uart_2nd, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                    m_cfg_board_testmode_tx_buf_uart_idx_2nd = m_cfg_board_testmode_rx_buf_uart_idx;
                    m_cfg_board_testmode_rx_buf_uart_idx = 0;
                    CRITICAL_REGION_EXIT();
                    for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx_2nd; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_tx_buf_uart_2nd[i]);
                    }
                    m_cfg_board_testmode_tx_buf_uart_idx_2nd = 0;
                    m_cfg_testmode_wait_tick = 0;
                }
                else  //sigfox to external (uart connected to external)
                {
                    CRITICAL_REGION_ENTER();
                    memcpy(m_cfg_board_testmode_tx_buf_uart, m_cfg_board_testmode_rx_buf_uart_2nd, m_cfg_board_testmode_rx_buf_uart_idx_2nd);
                    m_cfg_board_testmode_tx_buf_uart_idx = m_cfg_board_testmode_rx_buf_uart_idx_2nd;
                    m_cfg_board_testmode_rx_buf_uart_idx_2nd = 0;
                    CRITICAL_REGION_EXIT();
                    for(i=0; i<m_cfg_board_testmode_tx_buf_uart_idx; i++)
                    {
                        app_uart_put(m_cfg_board_testmode_tx_buf_uart[i]);
                    }
                    m_cfg_board_testmode_tx_buf_uart_idx = 0;
                }
                bridge_from_uart_to_uart_flag_old = m_cfg_bridge_from_uart_to_uart_flag;
            }
            
            if(m_cfg_bridge_from_uart_to_uart_flag)  //wait sigfox resp
            {
                if(m_cfg_testmode_wait_tick >= 1)
                {
                    if(++m_cfg_testmode_wait_tick > 5)  //timeout
                    {
                        m_cfg_bridge_from_uart_to_uart_flag = false; //wait external input
                        m_cfg_testmode_wait_tick = 0;
                    }
                }
            }
            nrf_delay_ms(100);
        }
    }
}

uint32_t UART2NUS_uartRxBufSize = 0;
static void cfg_board_testmode_UART2NUS_Recv_handler(const uint8_t * p_data, uint16_t length)
{
    int i;
    if(length == 2 && p_data[0] == 'C' && p_data[1] == 'F')
    {
        cfg_nvm_factory_reset(true);  //factory reset work
    }
    if(length > 0)
    {
        for(i=0; i<length; i++)
        {
            app_uart_put(p_data[i]);
        }
    }
}

static void cfg_board_bridge_from_NUS_to_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&uart_byte));
            CRITICAL_REGION_ENTER();
            if(m_cfg_board_testmode_rx_buf_uart_idx < (UART2NUS_uartRxBufSize))
            {
                m_cfg_board_testmode_rx_buf_uart[m_cfg_board_testmode_rx_buf_uart_idx++] = uart_byte;
            }
            CRITICAL_REGION_EXIT();
            break;

        case APP_UART_COMMUNICATION_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "NUS_to_uart Commu Err!\n");
            break;

        case APP_UART_FIFO_ERROR:
//            cPrintLog(CDBG_FCTRL_INFO, "NUS_to_uart fifo Err!\n");
            break;

        default:
            break;
    }
}


static void cfg_board_bridge_from_NUS_to_uart(uint32_t baud_rate, uint32_t rx_pin_no, uint32_t tx_pin_no)
{
    uint32_t                     err_code;
    app_uart_comm_params_t comm_params =
    {
        PIN_DEF_DTM_RX, //RX_PIN_NUMBER,
        PIN_DEF_DTM_TX,  //TX_PIN_NUMBER,
        UART_PIN_DISCONNECTED,  //RTS_PIN_NUMBER, //
        UART_PIN_DISCONNECTED,  //CTS_PIN_NUMBER, //
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    unsigned rtt_rd_size;
    char rtt_rd_bufffer[64];   //check BUFFER_SIZE_DOWN
    int i;
    uint32_t rtt_read_time_out=0;
    uint8_t device_name_buf[32];
    uint32_t device_name_len;
    int firstSendBleFlag = 0;
    int idx, size;

    SEGGER_RTT_printf(0, "cfg_board_bridge_from_NUS_to_uart baud rate:%08x, rx:%d, tx:%d\n", baud_rate, rx_pin_no, tx_pin_no);
    comm_params.baud_rate = baud_rate;
    comm_params.rx_pin_no = rx_pin_no;
    comm_params.tx_pin_no = tx_pin_no;

    //Since the TBC can not be executed, it uses TBC buffers.
    if((2048 <= CTBC_TX_BUF_SIZE))
    {
        extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
        extern uint8_t m_cTBC_bypasscmd_buf[CTBC_BYPASS_CMD_BUF_SIZE];
        app_uart_buffers_t buffers;
        UART2NUS_uartRxBufSize = CTBC_TX_BUF_SIZE/2;
        err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
        cfg_ble_stack_init(NULL);
        
        cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);
        device_name_len = sprintf((char *)device_name_buf, "SFX2NUS_%02x%02x", m_module_peripheral_ID.ble_MAC[4], m_module_peripheral_ID.ble_MAC[5]);
        cfg_ble_gap_params_init((const uint8_t *)device_name_buf, device_name_len);
        cfg_ble_peer_manager_init(false);
        cfg_ble_gatt_init();
        cfg_ble_services_init(false, true, cfg_board_testmode_UART2NUS_Recv_handler);
        cfg_ble_advertising_init(MSEC_TO_UNITS(100, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, NULL);
        cfg_ble_conn_params_init();

        m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[0];

        //ref APP_UART_FIFO_INIT()
        buffers.rx_buf      = &m_cTBC_bypasscmd_buf[0];
        buffers.rx_buf_size = 128;
        buffers.tx_buf      = &m_cTBC_bypasscmd_buf[128];
        buffers.tx_buf_size = 128;
        err_code = app_uart_init(&comm_params, &buffers, cfg_board_bridge_from_NUS_to_uart_event_handle, APP_IRQ_PRIORITY_LOW);
        APP_ERROR_CHECK(err_code);

        m_cfg_board_testmode_tx_buf_uart_2nd = &m_cTBC_tx_buf[UART2NUS_uartRxBufSize];

        cfg_ble_advertising_start();

        while(1)
        {
            if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_tx_buf_uart_2nd, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                m_cfg_board_testmode_tx_buf_uart_idx_2nd = m_cfg_board_testmode_rx_buf_uart_idx;
                m_cfg_board_testmode_rx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
                if(m_cfg_board_testmode_tx_buf_uart_idx_2nd > 0)
                {
                    if(ble_connect_on)
                    {
                        if(!firstSendBleFlag)
                        {
                            nrf_delay_ms(100);
                            SEGGER_RTT_printf(0, "====Nus Send====\n");
                            firstSendBleFlag = 1;
                        }
                        idx = 0;
                        while(m_cfg_board_testmode_tx_buf_uart_idx_2nd-idx > 0)
                        {
                            size = ((m_cfg_board_testmode_tx_buf_uart_idx_2nd-idx) > 20)?20:(m_cfg_board_testmode_tx_buf_uart_idx_2nd-idx);
                            do
                            {
                                err_code = cfg_ble_nus_data_send(&m_cfg_board_testmode_tx_buf_uart_2nd[idx], size);
                                if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                    (err_code != NRF_ERROR_RESOURCES) &&
                                    (err_code != NRF_ERROR_NOT_FOUND))
                                {
                                    if(err_code != NRF_SUCCESS)
                                        SEGGER_RTT_printf(0, "GPS2NUS Tx Err:%d\n", err_code);
                                }
                            } while (err_code == NRF_ERROR_RESOURCES);
                            idx += size;
                        }
                    }
                }
                m_cfg_board_testmode_tx_buf_uart_idx_2nd = 0;
            }
            cfg_board_RTT_read_N_clearCmd_proc();
         }
    }
    else
    {
        SEGGER_RTT_printf(0, "CTBC_TX_BUF_SIZE(%d) too small!\n", CTBC_TX_BUF_SIZE);
        while(1)
        {
            cfg_board_RTT_read_N_clearCmd_proc();
        }
    }
}

static void cfg_board_testmode_ble(void)
{
    SEGGER_RTT_printf(0, "====enter ble dtm mode====\n");
    dtm_mode();
}

static void cfg_board_testmode_sigfox(void)
{

    SEGGER_RTT_printf(0, "====enter sigfox uart over RTT mode====\n");
    
    //power enable pin control   
    cfg_board_common_power_control(module_comm_pwr_sigfox, true);
    nrf_delay_ms(10);
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
    nrf_delay_ms(10);  //spec is 4ms
//    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(10);
//    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(200);

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
    cfg_board_bridge_from_RTT_to_uart(UART_BAUDRATE_BAUDRATE_Baud115200, PIN_DEF_SIGFOX_UART_RX, PIN_DEF_SIGFOX_UART_TX);  //CDEV_SIGFOX_MONARCH_MODULE
#else
    cfg_board_bridge_from_RTT_to_uart(UART_BAUDRATE_BAUDRATE_Baud9600, PIN_DEF_SIGFOX_UART_RX, PIN_DEF_SIGFOX_UART_TX);                
#endif  
}

static void cfg_board_testmode_SFX2NUS(void)
{

    SEGGER_RTT_printf(0, "====enter bypass sigfox to NUS mode====\n");
    
    //power enable pin control   
    cfg_board_common_power_control(module_comm_pwr_sigfox, true);
    nrf_delay_ms(10);
    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
    nrf_delay_ms(10);  //spec is 4ms
    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(10);
    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(200);

#if defined(CDEV_SIGFOX_MONARCH_MODULE)
    cfg_board_bridge_from_NUS_to_uart(UART_BAUDRATE_BAUDRATE_Baud115200, PIN_DEF_SIGFOX_UART_RX, PIN_DEF_SIGFOX_UART_TX);  //CDEV_SIGFOX_MONARCH_MODULE
#else
    cfg_board_bridge_from_NUS_to_uart(UART_BAUDRATE_BAUDRATE_Baud9600, PIN_DEF_SIGFOX_UART_RX, PIN_DEF_SIGFOX_UART_TX);
#endif  
}


#ifdef FEATURE_CFG_CHECK_NV_BOOT_MODE
#ifdef CDEV_WIFI_MODULE
static void cfg_board_testmode_wifi_reset_detect(const uint8_t *pData, uint32_t dataSize)
{
    if(dataSize == 9 && (strncmp((const char*)pData, (const char*)"\r\nready\r\n",9) == 0))
    {
        cPrintLog(CDBG_MAIN_LOG, "Wifi reset!\n");
        cfg_board_reset();  //reset work
    }
}
#endif

static void cfg_board_testmode_wifi_AP_N_ble(void)
{
    uint32_t                err_code;
    uint8_t device_name_buf[32];
    uint32_t device_name_len;

    cfg_ble_led_control(true);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    cfg_ble_stack_init(NULL);   
    cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);

#ifdef CDEV_WIFI_MODULE
    int wifi_result;
    int timeout;
    uint8_t cmd_buf[128];
    unsigned int cmd_buf_idx;
    
    //disable wifi info log
    CDBG_mask_clear(CDBG_NUM2MASK(CDBG_WIFI_INFO));
    cWifi_prepare_start(m_module_peripheral_ID.wifi_MAC_STA);  // prepare for WIFI module

    wifi_result = cWifi_bypass_req(cfg_board_testmode_wifi_reset_detect, NULL);
    if(wifi_result == CWIFI_Result_OK)
    {
        timeout = 5000;
        while(!cWifiState_is_bypass_mode())
        {
            if(--timeout==0)break;  //wait bypassmode
            nrf_delay_ms(1);
        }
        if(timeout > 0)
        {
            cmd_buf_idx = sprintf((char *)cmd_buf, "ATE0");
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            //wait bypass ready
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(500);  //wait for response

            cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CWMODE_CUR=2");
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            //wait bypass ready
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(1000);  //wait for response

            //make at command
            cmd_buf_idx = sprintf((char *)cmd_buf, "AT+CWSAP=\"SFMTEST%02x%02x\",\"1234567890\",5,3", m_module_peripheral_ID.wifi_MAC_STA[4], m_module_peripheral_ID.wifi_MAC_STA[5]);
            cmd_buf_idx += sprintf((char *)&cmd_buf[cmd_buf_idx], "\r\n");
            while(!cWifiState_is_bypass_ready());
            cWifiState_bypass_write_request((const char *)cmd_buf, (unsigned int)cmd_buf_idx);
            nrf_delay_ms(500);  //wait for response
        }
        else
        {
            cPrintLog(CDBG_MAIN_LOG, "Wifi bypassmode timeout!\n");
        }
    }
    else
    {
        // WIFI not available or busy
        cPrintLog(CDBG_MAIN_LOG, "Not Availalble Wifi Module!\n");
    }
#endif
    device_name_len = sprintf((char *)device_name_buf, "SFMTEST%02x%02x", m_module_peripheral_ID.ble_MAC[4], m_module_peripheral_ID.ble_MAC[5]);
    cfg_ble_advertising_init(MSEC_TO_UNITS(50, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, NULL);
    cfg_ble_gap_params_init((const uint8_t *)device_name_buf, device_name_len);
    cfg_ble_advertising_start();
}

static void cfg_board_ble_scan_print_handler(const ble_gap_evt_adv_report_t * p_adv_report)
{
    const uint8_t *p_d;
    uint16_t d_len;
    int i;

    p_d = p_adv_report->data.p_data;
    d_len = p_adv_report->data.len;
    cPrintLog(CDBG_MAIN_LOG,"Mac:%02x%02x%02x%02x%02x%02x Len:%02d ",
             p_adv_report->peer_addr.addr[0],
             p_adv_report->peer_addr.addr[1],
             p_adv_report->peer_addr.addr[2],
             p_adv_report->peer_addr.addr[3],
             p_adv_report->peer_addr.addr[4],
             p_adv_report->peer_addr.addr[5],
             d_len
             );
    cPrintLog(CDBG_MAIN_LOG,"Data:");
    for(i = 6; i < d_len; i++)
    {
        cPrintLog(CDBG_MAIN_LOG,"%02x", p_d[i]);
    }
    cPrintLog(CDBG_MAIN_LOG,"\r\n");
    (void)p_d; (void)d_len;
}

static void cfg_board_testmode_ble_scan(void)
{
    uint32_t                err_code;

    cfg_ble_led_control(true);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    cfg_ble_stack_init(NULL);   

    cfg_ble_scan_init();
    cfg_ble_scan_start(cfg_board_ble_scan_print_handler);
}

static void cfg_board_testmode_BLE_nus_echo_handler(const uint8_t * p_data, uint16_t length)
{
    if(length > 0)
    {
        cfg_ble_nus_data_send((uint8_t *)p_data, length);
    }
}

APP_TIMER_DEF(m_testmode_BLE_timer_id);
static cfg_board_LowPwr_Periodic_1Sec_CB_func m_cfg_LowPwr_periodic_func = NULL;

static void cfg_board_testmode_BLE_timer_handler(void * p_context)
{
    (void)p_context;
    if(m_cfg_LowPwr_periodic_func)m_cfg_LowPwr_periodic_func();
}

void cfg_board_testmode_BLE_Advertising_LowPwr(bool basic_resource_init, bool adv_start, cfg_board_LowPwr_test_func test_func, cfg_board_LowPwr_Periodic_1Sec_CB_func cb_func)
{
    uint32_t                err_code;
    uint8_t device_name_buf[32];
    uint32_t device_name_len;

    m_cfg_LowPwr_periodic_func = cb_func;
    if(basic_resource_init)
    {
        if(nrf_sdh_is_enabled())
        {
            cPrintLog(CDBG_MAIN_LOG, "Error! softdevice already Inited!\n");
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
        }
        // Initialize timer module.
        err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
        cfg_ble_stack_init(NULL);
        device_name_len = sprintf((char *)device_name_buf, "TestLowPwr");
        cfg_ble_gap_params_init((const uint8_t *)device_name_buf, device_name_len);
        cfg_ble_peer_manager_init(false);
        cfg_ble_gatt_init();
        cfg_ble_services_init(false, true, cfg_board_testmode_BLE_nus_echo_handler);
        cfg_ble_advertising_init(MSEC_TO_UNITS(1000, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, NULL);
        cfg_ble_conn_params_init();
    }

    if(m_cfg_shutdown_peripherals_func)m_cfg_shutdown_peripherals_func();
    if(adv_start)
    {
        if(!nrf_sdh_is_enabled())
        {
            cPrintLog(CDBG_MAIN_LOG, "Error! softdevice Not Inited!\n");
            APP_ERROR_CHECK(NRF_ERROR_SOFTDEVICE_NOT_ENABLED);
        }
        cfg_ble_advertising_start();
    }
    app_timer_create(&m_testmode_BLE_timer_id, APP_TIMER_MODE_REPEATED, cfg_board_testmode_BLE_timer_handler);
    if(cb_func)
    {
        app_timer_start(m_testmode_BLE_timer_id, APP_TIMER_TICKS(1000), NULL);
    }
    if(test_func)test_func();
    
    while(1)
    {
        cfg_board_power_manage();
        cfg_board_RTT_read_N_clearCmd_proc();
    }
}
#endif

#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
static int cfg_board_checked_i2cdbg_con = 0;
int cfg_board_check_bootstrap_checked_i2cdbg_connected(void)
{
    return cfg_board_checked_i2cdbg_con;
}
void cfg_board_check_bootstrap_pin(void)
{
    uint32_t pinLvl_DL_EN, pinLvl_I2C0_SCL_DBG,pinLvl_I2C0_SDA_DBG;

    uint32_t pinLvl_I2C0_SCL_DBG_old, pinLvl_I2C0_SDA_DBG_old;
    bool special_mode = false;
    int i;

    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN0, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN1, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(50);

    pinLvl_DL_EN = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);

    pinLvl_I2C0_SCL_DBG = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN0);
    pinLvl_I2C0_SDA_DBG = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN1);
    pinLvl_I2C0_SCL_DBG_old = pinLvl_I2C0_SCL_DBG;
    pinLvl_I2C0_SDA_DBG_old = pinLvl_I2C0_SDA_DBG;

    //i2c slave busy check
    for(i = 0; i<100; i++)
    {
        nrf_delay_us(1);
        pinLvl_I2C0_SCL_DBG = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN0);
        pinLvl_I2C0_SDA_DBG = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN1);
        if((pinLvl_I2C0_SCL_DBG_old != pinLvl_I2C0_SCL_DBG) || (pinLvl_I2C0_SDA_DBG_old != pinLvl_I2C0_SDA_DBG))
        {
            //i2c slave connected and working
            cPrintLog(CDBG_FCTRL_INFO, "bootstrap pin i2c slave connected and working\n");
            pinLvl_I2C0_SCL_DBG = 1;
            pinLvl_I2C0_SDA_DBG = 1;
            break;
        }
        pinLvl_I2C0_SCL_DBG_old = pinLvl_I2C0_SCL_DBG;
        pinLvl_I2C0_SDA_DBG_old = pinLvl_I2C0_SDA_DBG;
    }
    if((pinLvl_I2C0_SCL_DBG == 0 && pinLvl_I2C0_SDA_DBG == 1) || (pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 0))
    {
        special_mode = true;
    }
    else if(pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 1)
    {
        cfg_board_checked_i2cdbg_con = 1;
    }
    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);
    nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN0);
    nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN1);

    cPrintLog(CDBG_FCTRL_INFO, "bootstrap pin lvl: dl:%d, scl:%d, sda:%d\n", pinLvl_DL_EN, pinLvl_I2C0_SCL_DBG, pinLvl_I2C0_SDA_DBG);

    if(special_mode)cPrintLog(CDBG_FCTRL_INFO, "====Enter Special Boot Mode====\n");

    if(pinLvl_DL_EN == 0)
    {
        cfg_board_testmode_wifi_download();
    }
    else if(special_mode)
    {
#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB) && (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)
        uint32_t bootstrap_config0, bootstrap_config1;
        nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0, NRF_GPIO_PIN_PULLDOWN);
        nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1, NRF_GPIO_PIN_PULLDOWN);
        nrf_delay_ms(20);
        bootstrap_config0 = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0);
        bootstrap_config1 = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1);
        nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0);
        nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1);
        cPrintLog(CDBG_FCTRL_INFO, "bootstrap scl-0 sda-1 cfg1:%d, cfg2:%d\n", bootstrap_config0, bootstrap_config1);

        if(pinLvl_I2C0_SCL_DBG == 0 && pinLvl_I2C0_SDA_DBG == 1)
        {
            if(bootstrap_config0 == 0 && bootstrap_config1 == 0)
            {
                cfg_board_testmode_sigfox();
            }
            else if(bootstrap_config0 == 0 && bootstrap_config1 == 1)
            {
                cfg_board_testmode_ble();
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 0)
            {
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi();
#endif
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 1)
            {
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi_always_on();
#endif
            }
        }
        else if(pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 0)
        {
            if(bootstrap_config0 == 0 && bootstrap_config1 == 0)
            {
#ifdef CDEV_GPS_MODULE
                cfg_board_testmode_gps();
#endif
            }
            else if(bootstrap_config0 == 0 && bootstrap_config1 == 1)
            {
                //not assigned
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 0)
            {
                //not assigned
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 1)
            {
                //not assigned
            }
        }
#elif (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
        if(pinLvl_I2C0_SCL_DBG == 0 && pinLvl_I2C0_SDA_DBG == 1)
        {
            uint32_t bootstrap_config0, bootstrap_config1;
            nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1, NRF_GPIO_PIN_PULLDOWN);
            nrf_delay_ms(20);
            bootstrap_config0 = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0);
            bootstrap_config1 = nrf_gpio_pin_read(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1);
            nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0);
            nrf_gpio_cfg_default(USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1);
            cPrintLog(CDBG_FCTRL_INFO, "bootstrap scl-0 sda-1 cfg1:%d, cfg2:%d\n", bootstrap_config0, bootstrap_config1);
            if(bootstrap_config0 == 0 && bootstrap_config1 == 0)
            {
                //not assigned
            }
            else if(bootstrap_config0 == 0 && bootstrap_config1 == 1)
            {
                cfg_board_testmode_ble();
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 0)
            {
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi();
#endif
            }
            else if(bootstrap_config0 == 1 && bootstrap_config1 == 1)
            {
                //not assigned
            }
        }
        else if(pinLvl_I2C0_SCL_DBG == 1 && pinLvl_I2C0_SDA_DBG == 0)
        {
            //not assigned
        }
#endif
    }
}
#else
int cfg_board_check_bootstrap_checked_i2cdbg_connected(void)
{
    return 0;
}
#endif

void cfg_board_check_wifi_downloadmode(void)
{
    uint32_t pinLvl_DL_EN;
    nrf_gpio_cfg_input(PIN_DEF_WIFI_INT, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(1);

    pinLvl_DL_EN = nrf_gpio_pin_read(PIN_DEF_WIFI_INT);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);
    cPrintLog(CDBG_FCTRL_DBG, "wifi dl pin lvl:%d\n", pinLvl_DL_EN);

    if(pinLvl_DL_EN == 0)
    {
        cfg_board_testmode_wifi_download();
    }
}

#ifdef CDEV_GPS_MODULE
static void cfg_board_testmode_GPS2NUS_Recv_handler(const uint8_t * p_data, uint16_t length)
{
    int dataIdx;
    if(length == 2 && p_data[0] == 'C' && p_data[1] == 'F')
    {
        cfg_nvm_factory_reset(true);  //factory reset work
    }

    if(length > 0)
    {
//        SEGGER_RTT_printf(0, "Gps2Nus Recv %d 0x", length);
        dataIdx = 0;
        do
        {
            SEGGER_RTT_printf(0, "%02x", p_data[dataIdx]);
        }while(++dataIdx < length);
        SEGGER_RTT_printf(0, "\n");
        if(length < 128)
        {
            if(m_cfg_board_testmode_rx_buf_uart_idx == 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_rx_buf_uart, p_data, length);
                m_cfg_board_testmode_rx_buf_uart_idx = length;
                CRITICAL_REGION_EXIT();
            }
            else
            {
                SEGGER_RTT_printf(0, "Buf Busy\n");
            }
        }
        else
        {
            SEGGER_RTT_printf(0, "Data Too Big\n");
        }
    }
}

void cfg_board_testmode_GPS2NUS(void)
{
    uint32_t                err_code;
    uint8_t device_name_buf[32];
    uint32_t device_name_len;
    nrf_drv_spi_config_t spi_config;
    extern uint8_t m_cTBC_tx_buf[CTBC_TX_BUF_SIZE];
    int firstSendBleFlag = 0;
    int i;
    int idx, size;

    SEGGER_RTT_printf(0, "====enter GPS2NUS====\n");
    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    cfg_ble_stack_init(NULL);

    cfg_ble_get_ble_mac_address(m_module_peripheral_ID.ble_MAC);
    device_name_len = sprintf((char *)device_name_buf, "GPS2NUS_%02x%02x", m_module_peripheral_ID.ble_MAC[4], m_module_peripheral_ID.ble_MAC[5]);
    cfg_ble_gap_params_init((const uint8_t *)device_name_buf, device_name_len);
    cfg_ble_peer_manager_init(false);
    cfg_ble_gatt_init();
    cfg_ble_services_init(false, true, cfg_board_testmode_GPS2NUS_Recv_handler);
    cfg_ble_advertising_init(MSEC_TO_UNITS(100, UNIT_0_625_MS), BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED, NULL);
    cfg_ble_conn_params_init();
    cGps_gpio_init();
    cGps_power_control(1, 1);  //gps power on
    cfg_ble_advertising_start();

    memcpy(&spi_config, &m_spi_config_default, sizeof(nrf_drv_spi_config_t));
    spi_config.ss_pin   = PIN_DEF_GPS_SPI_CS;
    spi_config.miso_pin = PIN_DEF_GPS_SPI_MISO;
    spi_config.mosi_pin = PIN_DEF_GPS_SPI_MOSI;
    spi_config.sck_pin  = PIN_DEF_GPS_SPI_SCK;
    spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
    spi_config.mode = NRF_DRV_SPI_MODE_0;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_cfg_board_testmode_gps_Spi, &spi_config, cfg_board_testmode_gps_spi_event_handler, NULL));
    m_cfg_board_testmode_gps_spi_tx_buf = &m_cTBC_tx_buf[0];
    m_cfg_board_testmode_gps_spi_rx_buf = &m_cTBC_tx_buf[128];
    m_cfg_board_testmode_tx_buf_uart = &m_cTBC_tx_buf[256];
    m_cfg_board_testmode_rx_buf_uart = &m_cTBC_tx_buf[384];
    memset(m_cfg_board_testmode_gps_spi_tx_buf, 0xff, 128);
    memset(m_cfg_board_testmode_gps_spi_rx_buf, 0xff, 128);
    m_cfg_board_testmode_gps_spi_XferDone = false;
    nrf_drv_spi_transfer(&m_cfg_board_testmode_gps_Spi, m_cfg_board_testmode_gps_spi_tx_buf, 128, m_cfg_board_testmode_gps_spi_rx_buf, 128);

    while(1)
    {
        if(m_cfg_board_testmode_gps_spi_XferDone)
        {
            memset(m_cfg_board_testmode_tx_buf_uart, 0, 128);
            m_cfg_board_testmode_tx_buf_uart_idx = 0;
            for(i=0; i<128; i++)
            {
                if(m_cfg_board_testmode_gps_spi_rx_buf[i] != 0xff)
                {
                    m_cfg_board_testmode_tx_buf_uart[m_cfg_board_testmode_tx_buf_uart_idx++] = m_cfg_board_testmode_gps_spi_rx_buf[i];
                }
            }
            if(m_cfg_board_testmode_tx_buf_uart > 0)
            {
                SEGGER_RTT_printf(0, "%s", m_cfg_board_testmode_tx_buf_uart);
                if(ble_connect_on)
                {
                    if(!firstSendBleFlag)
                    {
                        nrf_delay_ms(100);
                        SEGGER_RTT_printf(0, "====Nus Send====\n");
                        firstSendBleFlag = 1;
                    }
                    idx = 0;
                    while(m_cfg_board_testmode_tx_buf_uart_idx-idx > 0)
                    {
                        size = ((m_cfg_board_testmode_tx_buf_uart_idx-idx) > 20)?20:(m_cfg_board_testmode_tx_buf_uart_idx-idx);
                        do
                        {
                            err_code = cfg_ble_nus_data_send(&m_cfg_board_testmode_tx_buf_uart[idx], size);
                            if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                (err_code != NRF_ERROR_RESOURCES) &&
                                (err_code != NRF_ERROR_NOT_FOUND))
                            {
                                if(err_code != NRF_SUCCESS)
                                    SEGGER_RTT_printf(0, "GPS2NUS Tx Err:%d\n", err_code);
                            }
                        } while (err_code == NRF_ERROR_RESOURCES);
                        idx += size;
                    }
                }
            }
            memset(m_cfg_board_testmode_gps_spi_tx_buf, 0xff, 128);
            memset(m_cfg_board_testmode_gps_spi_rx_buf, 0xff, 128);
            if(m_cfg_board_testmode_rx_buf_uart_idx > 0)
            {
                CRITICAL_REGION_ENTER();
                memcpy(m_cfg_board_testmode_gps_spi_tx_buf, m_cfg_board_testmode_rx_buf_uart, m_cfg_board_testmode_rx_buf_uart_idx);
                m_cfg_board_testmode_rx_buf_uart_idx = 0;
                CRITICAL_REGION_EXIT();
            }
            m_cfg_board_testmode_gps_spi_XferDone = false;
            nrf_drv_spi_transfer(&m_cfg_board_testmode_gps_Spi, m_cfg_board_testmode_gps_spi_tx_buf, 128, m_cfg_board_testmode_gps_spi_rx_buf, 128);
        }
        cfg_board_RTT_read_N_clearCmd_proc();
    }
}
#endif
void cfg_board_check_bootmode_in_flash(void)
{
#ifdef FEATURE_CFG_CHECK_NV_BOOT_MODE
    int bootmode;

    if(module_parameter_get_bootmode(&bootmode))
    {
        cPrintLog(CDBG_MAIN_LOG, "Enter BMODE_FLS : %d\n", bootmode);
        switch(bootmode)
        {
            case 1:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;
            case 2:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_WIFI_MODULE
                cfg_board_testmode_wifi_always_on();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;

            case 3:
                cTBC_setting_erase_wait_for_testmode(3);
                cfg_board_testmode_ble();
                break;

            case 4:
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_GPS_MODULE
                cfg_board_testmode_gps();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;
            case 5:
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter wifi rf test bridge_from_RTT_to_uart mode====\n");
#ifdef CDEV_WIFI_MODULE
                cWifi_enter_rftest_mode();
#endif
                cfg_board_bridge_from_RTT_to_uart(UART_BAUDRATE_BAUDRATE_Baud115200, PIN_DEF_DTM_RX, PIN_DEF_DTM_TX);
                break;

            case 6:
                cTBC_setting_erase_wait_for_testmode(3);
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
                cfg_board_testmode_sigfox();
#else
                SEGGER_RTT_printf(0, "====Not Supported====\n");
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;

            case 7:
                cTBC_setting_erase_wait_for_testmode(3);
#if defined(CDEV_SIGFOX_MODULE)
                SEGGER_RTT_printf(0, "====enter sigfox over Uart====\n");
                
                //power enable pin control   
                cfg_board_common_power_control(module_comm_pwr_sigfox, true);
                nrf_delay_ms(10);
                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
                nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 1);
                nrf_delay_ms(10);  //spec is 4ms
//                nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
                nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
                nrf_delay_ms(10);
//                nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 1);
                nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLUP);
                nrf_delay_ms(50);

                cfg_board_sigfox_bridge_from_uart(UART_BAUDRATE_BAUDRATE_Baud9600, PIN_DEF_DTM_RX, PIN_DEF_DTM_TX);
#else
                SEGGER_RTT_printf(0, "====Not Supported====\n");
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;

            case 8:  //WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter WIFI AP(SFMTEST0000) and BLE BEACON(SFMTEST0000)====\n");
                cfg_board_testmode_wifi_AP_N_ble();
                cfg_board_RTT_reset_N_factory_reset_proc();
                break;

            case 9:  //BLE Scan
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter BLE Scan====\n");
                cfg_board_testmode_ble_scan();
                cfg_board_RTT_reset_N_factory_reset_proc();
                break;

            case 10:  //BLE Advertising
                cTBC_setting_erase_wait_for_testmode(3);
                SEGGER_RTT_printf(0, "====enter BLE Advertising====\n");
                cfg_board_testmode_BLE_Advertising_LowPwr(true, true, NULL, NULL);
                break;

            case 11:  //Bypass for GPS to Nus  //CMB GPS2NUS mode
                cTBC_setting_erase_wait_for_testmode(3);
#ifdef CDEV_GPS_MODULE
                cfg_board_testmode_GPS2NUS();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;

            case 12:  //Bypass for Sigfox to Nus  //CMC SFX2NUS mode
                cTBC_setting_erase_wait_for_testmode(3);
#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
                cfg_board_testmode_SFX2NUS();
#else
                cfg_board_RTT_reset_N_factory_reset_proc();
#endif
                break;

            default:
                break;
        }
    }
#endif
}

cfg_bmode_reg_user_defined_func bmode_reg_user_def_func = NULL;

void cfg_board_set_bootmode_in_register_with_ble(boot_mode_in_reg_e mode)
{
    uint32_t gpregretVal;
    uint32_t err_code;

    if(mode == bmode_reg_factory_test_with_ble)
        gpregretVal = BMODE_REG_GPREGRET | BMODE_REG_MODE_FACTORY_TEST_WITH_BLE;
    else if(mode == bmode_reg_enter_deep_sleep)
        gpregretVal = BMODE_REG_GPREGRET | BMODE_REG_MODE_ENTER_DEEP_SLEEP;
    else if(mode == bmode_reg_user_defined)
        gpregretVal = BMODE_REG_GPREGRET | BMODE_REG_MODE_USER_DEFINED;
    else
        gpregretVal = 0;
    cPrintLog(CDBG_MAIN_LOG, "bmode_reg to %d [0x%02x]\n", mode, gpregretVal);
    sd_power_gpregret_clr(0, 0xffffffff);
    sd_power_gpregret_set(0, gpregretVal);
}

void cfg_board_check_bootmode_in_register(void)
{
    uint8_t             gpregretVal;
    boot_mode_in_reg_e  mode;

    gpregretVal = nrf_power_gpregret_get();

    if((gpregretVal & BMODE_REG_GPREGRET_MASK) == BMODE_REG_GPREGRET)
    {
        nrf_power_gpregret_set(0);
        if((gpregretVal & BMODE_REG_MODE_BIT_MASK) == BMODE_REG_MODE_FACTORY_TEST_WITH_BLE)
            mode = bmode_reg_factory_test_with_ble;
        else if((gpregretVal & BMODE_REG_MODE_BIT_MASK) == BMODE_REG_MODE_ENTER_DEEP_SLEEP)
            mode = bmode_reg_enter_deep_sleep;
        else if((gpregretVal & BMODE_REG_MODE_BIT_MASK) == BMODE_REG_MODE_USER_DEFINED)
            mode = bmode_reg_user_defined;
        else
            mode = bmode_reg_none;
        
        switch(mode)
        {
            case bmode_reg_factory_test_with_ble:
#ifdef FEATURE_CFG_FULL_TEST_CMD_WITH_BLE_NUS
                cPrintLog(CDBG_MAIN_LOG, "bmode_reg_full_test_mode enter!\n");
                cfg_full_test_mode_enter();
#endif
                break;
            case bmode_reg_enter_deep_sleep:
                cPrintLog(CDBG_MAIN_LOG, "bmode_reg_deep_sleep enter!\n");
                cfg_board_prepare_power_down();
                cfg_board_goto_power_down();
                while(1);
                break;
            case bmode_reg_user_defined:
                if(bmode_reg_user_def_func)
                {
                    bmode_reg_user_def_func();
                }
                break;
            default:
                cPrintLog(CDBG_FCTRL_ERR, "Unknown BMODE_REG : %02x[%d]\n", gpregretVal, (int)mode);
                break;
        }
    }
}

