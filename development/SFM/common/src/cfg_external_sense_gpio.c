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
 * @brief tracking Sample Application cfg_external_sense_gpio.c file.
 *
 * This file contains the source code for an tracking sample application.
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

#include "cfg_dbg_log.h"
#include "cfg_config.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_opt3001_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_nus_cmd_proc.h"

static bool m_magnetic_old_status;
static uint32_t m_magnetic_detect_tick;
static nrf_drv_gpiote_pin_t m_magnetic_pin;
static bool m_magnetic_is_active_high;
static magnetic_attach_callback m_magnetic_attach_CB = NULL;
static magnetic_detach_callback m_magnetic_detach_CB = NULL;

static nrf_drv_gpiote_pin_t m_wkup_pin;
static wkup_detect_callback m_wkup_detect_CB = NULL;

static nrf_drv_gpiote_pin_t m_button_pin;

static int m_button_press_tick;
static bool m_button_sense_started = false;
static bool m_button_is_active_high;

static wkup_detect_callback m_button_short_press_CB = NULL;
static wkup_detect_callback m_button_long_press_CB = NULL;

extern bool m_hitrun_test_flag;

APP_TIMER_DEF(m_magnetic_timer_id);
APP_TIMER_DEF(m_button_id);

static bool cfg_magnetic_status_set(bool status)
{
    bool updated = false;
    if(status != m_magnetic_old_status)
    {
        if(status)
        {
            cPrintLog(CDBG_EXT_SEN_INFO, "%s The magnet is attached!\n", __func__);
            cTBC_write_state_noti("MagnetAttached");
            if(m_magnetic_attach_CB)m_magnetic_attach_CB();
#ifdef FEATURE_CFG_BLE_UART_CONTROL
            m_nus_service_parameter.magnet_event = '1';
#endif
            if(m_magnetic_is_active_high)
            {
                //It is high current consumption due to pulldown
                uint32_t pin_number = m_magnetic_pin;
                NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
                reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_PULL_Msk;
                reg->PIN_CNF[pin_number] |= (NRF_GPIO_PIN_NOPULL << GPIO_PIN_CNF_PULL_Pos);
            }
        }
        else
        {
            cPrintLog(CDBG_EXT_SEN_INFO, "%s The magnet is detached!\n", __func__);
            cTBC_write_state_noti("MagnetDetached");
            if(m_magnetic_detach_CB)m_magnetic_detach_CB();
            if(m_magnetic_is_active_high)
            {
                //It is Hi-Impedance
                uint32_t pin_number = m_magnetic_pin;
                NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
                reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_PULL_Msk;
                reg->PIN_CNF[pin_number] |= (NRF_GPIO_PIN_PULLDOWN << GPIO_PIN_CNF_PULL_Pos);
            }
        }
        m_magnetic_old_status = status;
        updated = true;
    }
    return updated;
}

bool cfg_magnetic_get_status(void)
{
    bool magnetic_status;
    if(m_magnetic_is_active_high)
    {
        magnetic_status = nrf_drv_gpiote_in_is_set(m_magnetic_pin);
    }
    else
    {
        magnetic_status = !nrf_drv_gpiote_in_is_set(m_magnetic_pin);
    }
    return magnetic_status;
}

bool cfg_magnetic_get_old_status(void) // 0:open, 1:close
{
    return m_magnetic_old_status;
}

static void cfg_magnetic_timeout_handler(void * p_context)
{
    bool magnetic_status;
    if(m_magnetic_is_active_high && m_magnetic_old_status)
    {
        ////polling for Hi-Impedance
        uint32_t pin_number = m_magnetic_pin;        
        NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
        reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_PULL_Msk;
        reg->PIN_CNF[pin_number] |= (NRF_GPIO_PIN_PULLDOWN << GPIO_PIN_CNF_PULL_Pos);
        magnetic_status = cfg_magnetic_get_status();
        reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_PULL_Msk;
        reg->PIN_CNF[pin_number] |= (NRF_GPIO_PIN_NOPULL << GPIO_PIN_CNF_PULL_Pos);

        if(!magnetic_status)
        {           

            cfg_magnetic_status_set(magnetic_status);
            nrf_drv_gpiote_in_event_enable(m_magnetic_pin, true);
        }
        else
        {
            app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_POLLING_MS), NULL);
        }
    }
    else
    {
        magnetic_status = cfg_magnetic_get_status();
        if(m_magnetic_old_status != magnetic_status)
        {
            if(++m_magnetic_detect_tick > GFG_MAGNETIC_DETECT_IGNORE_TICK_CNT)
            {
                cfg_magnetic_status_set(magnetic_status);
                if(m_magnetic_is_active_high && m_magnetic_old_status)
                {
                    app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_POLLING_MS), NULL);
                    cPrintLog(CDBG_EXT_SEN_INFO, "magnetic polling start!\n");
                }
                else
                {
                    nrf_drv_gpiote_in_event_enable(m_magnetic_pin, true);
                }
            }
            else
            {
                app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_DETECT_TICK_TIME_MS), NULL);
            }
        }
        else
        {
            nrf_drv_gpiote_in_event_enable(m_magnetic_pin, true);
        }
    }
}

static void cfg_magnetic_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    bool magnetic_status;
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_TOGGLE:
            if(pin == m_magnetic_pin)
            {
                magnetic_status = cfg_magnetic_get_status();
                if(m_magnetic_old_status != magnetic_status)
                {
                    //polling start
                    m_magnetic_detect_tick = 0;
                    nrf_drv_gpiote_in_event_disable(m_magnetic_pin);
                    app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_DETECT_TICK_TIME_MS), NULL);
                }
            }
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "%s bad pin:%d, action:%d\n", __func__, pin, action);
            break;
    }
}

/**
 * @brief Function for initializing the Magnetic Sensor. (is_active_high true: attached high signal, is_active_high false: attached low signal)
 * @param[in]   callback callback function for magnetic attacked
 */
void cfg_magnetic_sensor_init(bool is_active_high, uint32_t pin, magnetic_attach_callback callback_attach, magnetic_detach_callback callback_detach)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    m_magnetic_pin = pin;
    m_magnetic_is_active_high = is_active_high;

    if(m_magnetic_is_active_high)
    {
        //it is Hi-Impedance when the magnet is open. (close:high, internal pulldown is about 13 kOhm (11~16))
        config.pull = NRF_GPIO_PIN_PULLDOWN;  //pull up or pull down or no pull
    }
    else
    {
        config.pull = NRF_GPIO_PIN_PULLUP;  //pull up or pull down or no pull
    }

    err_code = app_timer_create(&m_magnetic_timer_id, APP_TIMER_MODE_SINGLE_SHOT, cfg_magnetic_timeout_handler);
    APP_ERROR_CHECK(err_code);

    m_magnetic_attach_CB = callback_attach;
    m_magnetic_detach_CB = callback_detach;
    err_code = nrf_drv_gpiote_in_init(m_magnetic_pin, &config, cfg_magnetic_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {       
        m_magnetic_old_status = cfg_magnetic_get_status();
        if(m_magnetic_is_active_high && m_magnetic_old_status)
        {
            //polling start
            uint32_t pin_number = m_magnetic_pin;
            NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_number);
            reg->PIN_CNF[pin_number] &= ~GPIO_PIN_CNF_PULL_Msk;
            reg->PIN_CNF[pin_number] |= (NRF_GPIO_PIN_NOPULL << GPIO_PIN_CNF_PULL_Pos);
            app_timer_start(m_magnetic_timer_id, APP_TIMER_TICKS(GFG_MAGNETIC_POLLING_MS), NULL);
        }
        else
        {
            //interrupt start
            nrf_drv_gpiote_in_event_enable(m_magnetic_pin, true);
        }
        cPrintLog(CDBG_EXT_SEN_INFO, "magnetic sensor started! pin:%d, lvl:%d, state:%d\n", m_magnetic_pin, m_magnetic_is_active_high, m_magnetic_old_status);
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__,err_code);
    }

}

/**
 * @brief Callback function for handling interrupt from temperature.
 */
extern volatile bool main_EXTSEN_ISR_detected;
#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
void tmp10x_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

    if(!m_hitrun_test_flag)
    {
        if(cfg_scen_check_sleep_state())
        {
             nrf_drv_gpiote_in_event_disable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN);
             tmp102_intr_lowhigh_clear_state();
        }
    }
    cTBC_event_noti("EXTSEN_ISR_TMP");
    main_EXTSEN_ISR_detected = true;
}
#endif

/**
 * @brief  function for setting interrupt from temperature.
 */
void cfg_tmp10x_interrupt_init(void)
{
#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, &in_config, tmp10x_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, true);
#endif
}

#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
/**
 * @brief Callback function for handling interrupt from ALS.
 */
void opt3001_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

    if(!m_hitrun_test_flag)
    {
        if(cfg_scen_check_sleep_state())
        {
            nrf_drv_gpiote_in_event_disable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN);
            opt3001_set_clear_state();
        }
    }
    cTBC_event_noti("EXTSEN_ISR_OPT");
    main_EXTSEN_ISR_detected = true;
}
#endif

/**
 * @brief  function for setting interrupt from ALS.
 */
void cfg_opt3001_interrupt_init(void)
{
#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, &in_config, opt3001_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, true);
#endif
}

#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
/**
 * @brief Callback function for handling interrupt from ALS.
 */
void opt3001ntmp_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(!m_hitrun_test_flag)
    {
        if(cfg_scen_check_sleep_state())
        {
            nrf_drv_gpiote_in_event_disable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN);
            tmp102_intr_lowhigh_clear_state();
        }
        else
        {
            tmp102nopt3001_intr_clear_state();
        }
    }
    cTBC_event_noti("EXTSEN_ISR_OPT_TMP");
    main_EXTSEN_ISR_detected = true;
}
#endif

/**
 * @brief  function for setting interrupt from tmp & als.
 */
void cfg_tmp10xopt3001_interrupt_init(void)
{
#ifdef CDEV_EXTERNAL_I2C_SENSOR_INT_PIN
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, &in_config, opt3001ntmp_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(CDEV_EXTERNAL_I2C_SENSOR_INT_PIN, true);
#endif
}

/**
 * @brief Callback function for handling NFC events.
 */
static void cfg_wkup_gpio_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
            if(pin == m_wkup_pin)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "WKUP Signal Detected!\n");
                if(m_wkup_detect_CB)m_wkup_detect_CB();
            }
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "%s bad pin:%d, action:%d\n", __func__, pin, action);
            break;
    }
}

/**
 * @brief Function for initializing the wake up gpio. (active low signal)
 * @param[in]   callback callback function for wkup key push (active high)
 */
void cfg_wkup_gpio_init(uint32_t pin, wkup_detect_callback callback)   //support only active high
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config_wkup = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    m_wkup_pin = pin;
    config_wkup.pull = NRF_GPIO_PIN_PULLDOWN;  //pull up or pull down or no pull
    err_code = nrf_drv_gpiote_in_init(m_wkup_pin, &config_wkup, cfg_wkup_gpio_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {
        nrf_drv_gpiote_in_event_enable(m_wkup_pin, true);
        cPrintLog(CDBG_EXT_SEN_INFO, "wkup sense started! pin:%d\n", m_wkup_pin);
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__,err_code);
    }

    m_wkup_detect_CB = callback;
}

static void cfg_button_timer_start(void)
{
    app_timer_start(m_button_id, APP_TIMER_TICKS(GFG_BUTTON_SENSE_TICKE_MS), NULL);
}

static void cfg_button_sense_start(void)
{
    cPrintLog(CDBG_EXT_SEN_INFO, "button sense started, io:%d, lvl:%d\n", m_button_pin, m_button_is_active_high);
    m_button_sense_started = true;
    m_button_press_tick = 0;
    nrf_drv_gpiote_in_event_enable(m_button_pin, true);
}

static void cfg_button_timeout_handler(void * p_context)
{
    bool io_status;
    m_button_press_tick++;
    io_status = nrf_drv_gpiote_in_is_set(m_button_pin);

    if(!m_button_sense_started)
    {
        if(io_status == m_button_is_active_high)
        {
            cfg_button_timer_start();
        }
        else
        {
            cfg_button_sense_start();
        }
    }
    else
    {

        if(io_status == m_button_is_active_high)
        {
            //press key
            if(m_button_press_tick == GFG_BUTTON_SENSE_PRESS_TICK_LONG)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "BUTTON press long!\n");
                if(m_button_long_press_CB)m_button_long_press_CB();
            }
            cfg_button_timer_start();
        }
        else
        {
            //release key
            if(m_button_press_tick > GFG_BUTTON_SENSE_PRESS_TICK_SHORT && m_button_press_tick < GFG_BUTTON_SENSE_PRESS_TICK_LONG)
            {
                cPrintLog(CDBG_EXT_SEN_INFO, "BUTTON press short!\n");
                if(m_button_short_press_CB)m_button_short_press_CB();
            }
            m_button_press_tick = 0;
            nrf_drv_gpiote_in_event_enable(m_button_pin, true);
        }
    }
}

static void cfg_button_isr_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
        case NRF_GPIOTE_POLARITY_HITOLO:
            if(m_button_is_active_high)
            {
                if(action != NRF_GPIOTE_POLARITY_LOTOHI)
                {
                    cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr H\n");
                }
            }
            else
            {
                if(action != NRF_GPIOTE_POLARITY_HITOLO)
                {
                    cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr L\n");
                }
            }
            nrf_drv_gpiote_in_event_disable(m_button_pin);
            cfg_button_timer_start();
            break;

        default:
            cPrintLog(CDBG_EXT_SEN_ERR, "bad button isr pin:%d, action:%d\n", pin, action);
            break;
    }
}

void cfg_button_init(bool is_active_high, uint32_t pin, usr_def_button_short_press_callback short_press_CB, usr_def_button_long_press_callback long_press_CB)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t config_h = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    nrf_drv_gpiote_in_config_t config_l = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    nrf_drv_gpiote_in_config_t *config_ptr;
    bool io_status;

    m_button_pin = pin;
    m_button_short_press_CB = short_press_CB;
    m_button_long_press_CB = long_press_CB;
    m_button_is_active_high = is_active_high;

    if(m_button_is_active_high)
    {
        config_h.pull = NRF_GPIO_PIN_PULLDOWN;  //pull up or pull down or no pull
        config_ptr = &config_h;
    }
    else
    {
        config_l.pull = NRF_GPIO_PIN_PULLUP;  //pull up or pull down or no pull
        config_ptr = &config_l;
    }

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_drv_gpiote_in_init(m_button_pin, config_ptr, cfg_button_isr_event_handler);
    if(err_code==NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_DBG, "button sense init! pin:%d, active lvl:%d\n", m_button_pin, m_button_is_active_high);
        io_status = nrf_drv_gpiote_in_is_set(m_button_pin);
        err_code = app_timer_create(&m_button_id, APP_TIMER_MODE_SINGLE_SHOT, cfg_button_timeout_handler);
        APP_ERROR_CHECK(err_code);
        if(io_status == m_button_is_active_high)
        {
            m_button_sense_started = false;
            cfg_button_timer_start();
        }
        else
        {
            cfg_button_sense_start();
        }
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "%s nrf_drv_gpiote_in_event_enable error:%d\n", __func__, err_code);
    }
}
void cfg_button_release(void)
{
    nrf_drv_gpiote_in_event_disable(m_button_pin);
    nrf_drv_gpiote_in_uninit(m_button_pin);
    m_button_sense_started = false;
}


