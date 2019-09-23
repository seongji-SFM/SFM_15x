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
 * @brief tracking Sample Application cfg_opt3001_module.c file.
 *
 * This file contains the source code for an opt3001 ambient light sensor.
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

#include "cfg_dbg_log.h"
#include "cfg_config.h"
#include "cfg_twis_board_control.h"
#include "cfg_opt3001_module.h" 
#include "cfg_tmp102_module.h" 
#include "cfg_scenario.h"
#include "cfg_scenario.h"

opt3001_attach_callback m_opt3001_attach_CB = NULL;

extern volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/* TWI instance. */
extern const nrf_drv_twi_t m_twi;
    
//static uint8_t       m_tx_buf[20];    /**< TX buffer. */
static uint8_t       m_rx_buf[20];    /**< RX buffer. */

opt3001_state_s m_opt3001_state;
#define MPU_TWI_TIMEOUT     100000
extern uint8_t i2c_use_mode;

APP_TIMER_DEF(m_opt3001_timer_id); /** OPT3001 timer. */

#define CPRINTLOG_ALS false // register value log(default false)
uint32_t als_lux;

void opt3001_i2c_init(void)
{
//    cfg_i2c_master_uninit();
    if(!m_cfg_i2c_master_init_flag)
        cfg_i2c_master_init();
    nrf_delay_ms(10);
}

opt3001_state_s opt3001_get_state()
{
    return m_opt3001_state;
}

void opt3001_set_state(opt3001_state_s m_state)
{
    m_opt3001_state = m_state;
}

/** 
*  Read register from OPT3001 light sensor. 
*/ 
uint8_t opt3001_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt) 
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t stringpos = 0;

    spi_xfer_done = false;
    i2c_use_mode = 3;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, OPT3001_I2C_ADDR,&reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, OPT3001_I2C_ADDR, reg_data, cnt));
    while((!spi_xfer_done) && --timeout);
    if(!timeout){i2c_use_mode = 0; return NRF_ERROR_TIMEOUT;}
    for (stringpos = 0; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = reg_data[stringpos];
    }
    i2c_use_mode = 0;
    return NRF_SUCCESS;
} 
  
/** 
*  Write register to OPT3001 light sensor. 
*/ 
uint8_t opt3001_i2c_write(uint8_t reg_addr, uint16_t reg_data, uint8_t cnt) 
{ 
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t array[4];

    spi_xfer_done = false;
    i2c_use_mode = 3;
    timeout = MPU_TWI_TIMEOUT;
    array[0] = reg_addr;
    array[1] =((reg_data & 0xff00) >> 8);  //byte 1
    array[2] =(uint8_t)(reg_data & 0xff);  //byte 2

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, OPT3001_I2C_ADDR, array, 3, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout){i2c_use_mode = 0; return NRF_ERROR_TIMEOUT;}
    i2c_use_mode = 0;

    return NRF_SUCCESS;
}

int opt3001_get_reg_result(void) 
{
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_RESULT, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 RESULT read fail! \n",__LINE__);
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if ((m_rx_buf[0] != 0) || (m_rx_buf[1] != 0))
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get OPT3001_REG_RESULT[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    return opt_ret;
}

/** 
* Read OPT3001 light sensor data. 
*/ 
static int opt3001_get_manufacturer_id(void) 
{ 
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;
    
    ret = opt3001_i2c_read(OPT3001_REG_MAN_ID, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 MANUFACTURER ID read fail! \n",__LINE__);
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if (data == OPT3001_MANUFACTURER_ID)
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get OPT3001_REG_MAN_ID[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 MANUFACTURER ID value invalid ! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    return opt_ret;
}

/** 
* Read OPT3001 light sensor data. 
*/ 
static int opt3001_get_device_id(void) 
{ 
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_DEV_ID, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 DEVICE ID read fail! \n",__LINE__);
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if (data == OPT3001_DEVICE_ID)
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get OPT3001_REG_DEV_ID[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 DEVICE ID value invalid ! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    return opt_ret;
}

int opt3001_set_configuration(uint16_t config) 
{ 
    uint8_t ret; 
    int opt_ret = EC_ERROR_UNKNOWN;

    /* 
    * [15:12]: 0101b Automatic full scale (1310.40lux, 0.32lux/lsb) 
    * [11]   : 1b    Conversion time 800ms 
    * [10:9] : 10b   Continuous Mode of conversion operation 
    * [4]    : 1b    Latched window-style comparison operation 
    */ 
    ret = opt3001_i2c_write(OPT3001_REG_CONFIGURE, config, 3); 
    opt_ret = (int)ret;
    if(opt_ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 configuration write fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
    }
    return opt_ret;
}

int opt3001_get_configuration(void) 
{
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_CONFIGURE, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 configuration read fail! \n",__LINE__);
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if ((m_rx_buf[0] != 0) || (m_rx_buf[1] != 0))
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get init OPT3001_REG_CONFIGURE[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    return opt_ret;
}

static int opt3001_set_range(uint8_t range, int rnd) 
{ 
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    if (/*range < 0 || */range > OPT3001_RANGE_AUTOMATIC_FULL_SCALE) 
        return EC_ERROR_INVAL; 

    ret = opt3001_i2c_read(OPT3001_REG_CONFIGURE, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 configuration read fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if ((m_rx_buf[0] != 0) || (m_rx_buf[1] != 0))
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get OPT3001_REG_CONFIGURE[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 range value read fail! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
        return opt_ret;
    }

    ret = opt3001_i2c_write(OPT3001_REG_CONFIGURE, 
                            (data & OPT3001_RANGE_MASK) | 
                            (range << OPT3001_RANGE_OFFSET), 3); 
    opt_ret = (int)ret;
    if(opt_ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 range value write fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
    }
    return opt_ret; 
} 

static int opt3001_get_low_intr(uint16_t *low_int) 
{ 
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_INT_LIMIT_LSB, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 low limit read fail! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    else
    {
        data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
        if(low_int)*low_int = data;
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get low limit[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    return opt_ret;
}

static int opt3001_set_low_intr(uint16_t low) 
{ 
    uint8_t ret; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_write(OPT3001_REG_INT_LIMIT_LSB, low, 2);
    opt_ret = (int)ret;
    if(opt_ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 low limit write fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
    }
    return opt_ret;
}

static int opt3001_get_high_intr(uint16_t *high_int) 
{ 
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_INT_LIMIT_MSB, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 high limit read fail! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    else
    {
        data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
        if(high_int)*high_int = data;
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get high limit[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    return opt_ret;
}

static int opt3001_set_high_intr(uint16_t high) 
{ 
    uint8_t ret; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_write(OPT3001_REG_INT_LIMIT_MSB, high, 2);
    opt_ret = (int)ret;
    if(opt_ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 high limit write fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
    }
    return opt_ret;
}

int opt3001_set_shutdown(void) 
{

    uint8_t ret; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_set_configuration(DEFAULT_CONFIG_SHDWN);
    opt_ret = (int)ret;
    if(opt_ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 shutdown fail! \n",__LINE__);
        opt_ret = EC_ERROR_TIMEOUT;
    }
    return opt_ret;
}

/** 
* Read OPT3001 light sensor data. 
*/ 
int opt3001_get_lux(uint32_t *get_lux)
{ 
    uint8_t ret; 
    uint16_t data; 
    uint32_t lux;    
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_RESULT, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 lux read fail! \n",__LINE__);
        opt_ret = EC_ERROR_ACCESS_DENIED; 
        return opt_ret; 
    }
    else
    {
        data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Get OPT3001_REG_RESULT[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }

    /* 
     * The default power-on values will give 12 bits of precision: 
     * 0x0000-0x0fff indicates 0 to 1310.40 lux. We multiply the sensor 
     * value by a scaling factor to account for attenuation by glass, 
     * tinting, etc. 
     * lux = 2EXP[3:0] ¡¿ R[11:0] / 100 
     */ 
    lux = ((1 << ((data & 0xF000) >> 12)) * (data & 0x0FFF)) / 100; 
    if(get_lux)*get_lux = lux;
    if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] opt3001_get_lux lux[%d]\n",__LINE__, lux);

    return opt_ret; 
} 


void opt3001_set_clear_state(void) 
{
    opt3001_set_state(ALS_INTERRUPT_CLEAR_S);
    cfg_opt3001_timers_stop();
    cfg_opt3001_timers_start();
}

int opt3001_set_clear_intr(void) 
{
    uint8_t ret; 
    uint16_t data; 
    int opt_ret = EC_ERROR_UNKNOWN;

    ret = opt3001_i2c_read(OPT3001_REG_CONFIGURE, m_rx_buf, 2); 
    if(ret != NRF_SUCCESS)
    {
        opt_ret = EC_ERROR_TIMEOUT;
        cPrintLog(CDBG_EXT_SEN_ERR, "LINE[%d]  opt3001 configuration read fail! \n",__LINE__);
        return opt_ret;
    }
    data = ((m_rx_buf[0] << 8) & 0xFF00) | m_rx_buf[1];
    if ((m_rx_buf[0] != 0) || (m_rx_buf[1] != 0))
    {
        if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "LINE[%d] Clear OPT3001_REG_CONFIGURE[0x%02x]\n",__LINE__, data);
        opt_ret = EC_SUCCESS; 
    }
    else
    {
        opt_ret = EC_ERROR_ACCESS_DENIED; 
    }
    return opt_ret;
}

int32_t opt3001_interrupt_lux_value()
{
    int32_t ret_val = 0;

    switch (m_module_parameter.ambient_light_sensor_interrupt_value)
    {
        case OPT3001_RANGE_40P95_LUX:
            ret_val =  40; // 40 lux
            break;

        case OPT3001_RANGE_81P90_LUX:
            ret_val =  81; // 81 lux
            break;

        case OPT3001_RANGE_163P80_LUX:
            ret_val =  163; // 163 lux
            break;

        case OPT3001_RANGE_327P60_LUX:
            ret_val =  327; // 327 lux
            break;

        case OPT3001_RANGE_655P20_LUX:
            ret_val =  655; // 655 lux
            break;
            
        case OPT3001_RANGE_1310P40_LUX:
            ret_val =  1310; // 1310 lux
            break;
        
        case OPT3001_RANGE_2620P80_LUX:
            ret_val =  2620; // 2620 lux
            break;
        
        case OPT3001_RANGE_5241P60_LUX:
            ret_val =  5241; // 5241 lux
            break;
        
        case OPT3001_RANGE_10483P20_LUX:
            ret_val =  10483; // 10483 lux
            break;
        
        case OPT3001_RANGE_20966P40_LUX:
            ret_val =  20966; // 20966 lux
            break;
            
        case OPT3001_RANGE_41932P80_LUX:
            ret_val =  41932; // 41932 lux
            break;
        
        case OPT3001_RANGE_83865P60_LUX:
            ret_val =  83865; // 83865 lux
            break;
            
        default:
            ret_val = 0;
            break;
    }
return ret_val;
    
}

bool opt3001_get_enable_intr()
{
    bool ret_val = false;
 
    switch (m_module_parameter.ambient_light_sensor_interrupt_mode)
    {
        case ALS_INTR_SET_LOW:
        case ALS_INTR_SET_HIGH:
            ret_val =  true;
            break;
            
        default:
            ret_val = false;
            break;
    }
    return ret_val;
}

bool opt3001_get_enable_low()
{
    bool ret_val = false;

    switch (m_module_parameter.ambient_light_sensor_interrupt_mode)
    {
        case ALS_INTR_SET_LOW:
            ret_val =  true;
            break;
            
        default:
            ret_val = false;
            break;
    }
    return ret_val;
}

bool opt3001_get_enable_high()
{
    bool ret_val = false;

    switch (m_module_parameter.ambient_light_sensor_interrupt_mode)
    {
        case ALS_INTR_SET_HIGH:
            ret_val =  true;
            break;
            
        default:
            ret_val = false;
            break;
        }
    return ret_val;
}

bool opt3001_get_oldnew_int()
{
    bool ret = true;
    uint16_t low_int, high_int;
    uint16_t als_intr_lux = 0;
    
    opt3001_get_low_intr(&low_int);
    opt3001_get_high_intr(&high_int);

    als_intr_lux = ((m_module_parameter.ambient_light_sensor_interrupt_value << 12) | 0xfff);

    if(opt3001_get_enable_low())
    {
        if(low_int != als_intr_lux) 
            return false;
    }
    if(opt3001_get_enable_high())
    {
        if(high_int != als_intr_lux) 
            return false;
    }  
    return ret;
}

void opt3001_set_lowhigh_intr(void)
{
    uint16_t als_intr_lux = 0;
    uint16_t low_int, high_int;

    als_intr_lux = ((m_module_parameter.ambient_light_sensor_interrupt_value << 12) | 0xfff);

    if(opt3001_get_enable_low())
    {
        opt3001_set_low_intr(als_intr_lux);
    }
    else
    {
        opt3001_set_low_intr(OPT3001_INTR_LOW_DEFAULT);
    }

    if(opt3001_get_enable_high())
    {
        opt3001_set_high_intr(als_intr_lux);
    }
    else
    {
        opt3001_set_high_intr(OPT3001_INTR_HIGH_DEFAULT);
    }
  
    if(CPRINTLOG_ALS)
    {
        opt3001_get_low_intr(&low_int);
        opt3001_get_high_intr(&high_int);
    }
}

bool opt3001_intr_wakeup_reason(void) 
{
    bool ret = false;

    if(tmp102_get_enable_intr() && opt3001_get_enable_intr())
    {
        if((main_wakeup_reason == main_wakeup_reason_ambient_light_sensor_event_high)
            || (main_wakeup_reason == main_wakeup_reason_ambient_light_sensor_event_low)
            || (main_wakeup_reason == main_wakeup_reason_tmp_sensor_event_high)
            || (main_wakeup_reason == main_wakeup_reason_tmp_sensor_event_low))
        {
            ret = true;
        }
    }
    else if(opt3001_get_enable_intr())
    {
        if((main_wakeup_reason == main_wakeup_reason_ambient_light_sensor_event_high)
            || (main_wakeup_reason == main_wakeup_reason_ambient_light_sensor_event_low))
        {
            ret = true;
        }
    }
    return ret;
}

uint8_t opt3001_lux_compare_intr(uint32_t get_lux)
{
    uint8_t ret = ALS_INTR_NONE;
    uint16_t als_intr_lux = 0;
    uint16_t low_int, high_int;

    als_intr_lux = ((m_module_parameter.ambient_light_sensor_interrupt_value << 12) | 0xfff);
    if(opt3001_get_enable_low() && (get_lux <= opt3001_interrupt_lux_value()))
    {
        /* low interrupt - disable */
        als_intr_lux = OPT3001_INTR_LOW_DEFAULT;
        opt3001_set_low_intr(als_intr_lux);
        /* high interrupt */
        als_intr_lux = ((OPT3001_RANGE_327P60_LUX << 12) | 0xfff);        
        opt3001_set_high_intr(als_intr_lux);
        ret = ALS_INTR_LOW;
    }
    else if(opt3001_get_enable_high() && (get_lux >= opt3001_interrupt_lux_value()))
    {
        /* low interrupt */
        als_intr_lux = ((OPT3001_RANGE_81P90_LUX << 12) | 0xfff);        
        opt3001_set_low_intr(als_intr_lux);
        /* high interrupt - disable */
        als_intr_lux = OPT3001_INTR_HIGH_DEFAULT;
        opt3001_set_high_intr(als_intr_lux);
        
        ret = ALS_INTR_HIGH;
    }
    else
    {
        ret = ALS_INTR_NONE;
    }

    if(CPRINTLOG_ALS)
    {
        opt3001_get_low_intr(&low_int);
        opt3001_get_high_intr(&high_int);
    }
    return ret;
}

void opt3001_check_intr_reason(void) 
{
    uint32_t get_lux = 0;
    uint8_t als_int = 0;
    char display[40];
    
    if(cfg_scen_check_sleep_state())
    {
        opt3001_get_lux(&get_lux);
        als_lux = get_lux;

        if(opt3001_get_oldnew_int())
        {
            als_int = opt3001_lux_compare_intr(get_lux);
            if(als_int != ALS_INTR_NONE)
            {
                if(als_int == ALS_INTR_HIGH)
                {
                    cfg_scen_wakeup_request(main_wakeup_reason_ambient_light_sensor_event_high);
                }
                else if(als_int == ALS_INTR_LOW)
                {
                    cfg_scen_wakeup_request(main_wakeup_reason_ambient_light_sensor_event_low);
                }
                sprintf(display,"Ambient light sensor detected!!!\n");
                cPrintLog(CDBG_FCTRL_INFO,"%s", display);
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Ambient light sensor Lux:[%d] \n", __LINE__, als_lux);
            }
        }
        else
        {
            cPrintLog(CDBG_EXT_SEN_INFO, "%d Ambient light sensor Lux:[%d] \n", __LINE__, als_lux);
            opt3001_set_lowhigh_intr();      
        }
    }
}

void opt3001_get_information(void) 
{
    opt3001_get_manufacturer_id();
    opt3001_get_device_id();
}

uint32_t opt3001_send_General_Call_Reset(void)
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    uint8_t array[4];
    
    spi_xfer_done = false;
    i2c_use_mode = 3;
    timeout = MPU_TWI_TIMEOUT;
    array[0] = 0x06;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, 0, array, 1, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout){i2c_use_mode = 0; return NRF_ERROR_TIMEOUT;}
    i2c_use_mode = 0;
    
    return NRF_SUCCESS;
}

void opt3001_set_read_testmode(void)
{
    opt3001_set_configuration(DEFAULT_CONFIG_SET_100MS);   
    opt3001_set_range(OPT3001_RANGE_AUTOMATIC_FULL_SCALE, 0);
}
uint32_t opt3001_interrupt_test(void)
{
    return (uint32_t)opt3001_set_configuration((DEFAULT_CONFIG_SET_100MS | 0x0008));  //default and POL 1
}

void opt3001_init(void)
{
    
    opt3001_i2c_init();

    opt3001_get_manufacturer_id();

    opt3001_get_device_id();

    opt3001_get_reg_result();

    opt3001_get_configuration();

    opt3001_set_configuration(DEFAULT_CONFIG_SET_100MS);
    nrf_delay_ms(40);

    opt3001_set_range(OPT3001_RANGE_AUTOMATIC_FULL_SCALE, 0); 
}

static void opt3001_state_handler(void * p_context)
{
    uint32_t get_lux = 0;
    
    if(CPRINTLOG_ALS) cPrintLog(CDBG_EXT_SEN_INFO, "Line[%d] m_opt3001_state[%d] \n", __LINE__, m_opt3001_state);
    switch(m_opt3001_state)
    {
        case ALS_SET_S:
            als_lux = 0;
            opt3001_init();
            m_opt3001_state = ALS_SET_R;
            break;

        case ALS_SET_R:
            m_opt3001_state = ALS_INTERRUPT_S;
            break;

        case ALS_INTERRUPT_S:
            spi_xfer_done = false;
            opt3001_set_lowhigh_intr();
            m_opt3001_state = ALS_INTERRUPT_R;
            break;
            
        case ALS_INTERRUPT_R:
            m_opt3001_state = ALS_READ_DATA_S;
            break;

        case ALS_READ_DATA_S:
            {
                if(i2c_use_mode == 0)
                {
                    spi_xfer_done = false;
                    opt3001_get_lux(&get_lux);
                    als_lux = get_lux;
                    cPrintLog(CDBG_EXT_SEN_INFO, "%d Ambient light sensor Lux:[%d] \n", __LINE__, als_lux);
                    m_opt3001_state = ALS_READ_DATA_R;
                }
                else
                {
                    m_opt3001_state = ALS_READ_DATA_S;
                }
            }
            break;
            
        case ALS_READ_DATA_R:
            m_opt3001_state = EXIT_ALS;
            break;
            
        case ALS_READ_ONLY_S:
            if(i2c_use_mode == 0)
            {
                opt3001_i2c_init();
                opt3001_get_lux(&get_lux);
                als_lux = get_lux;
                cPrintLog(CDBG_EXT_SEN_INFO, "%d Ambient light sensor Lux:[%d] \n", __LINE__, als_lux);
                cfg_opt3001_timers_stop();
                m_opt3001_state = NONE_ALS;
            }
            else
            {
                m_opt3001_state = ALS_READ_ONLY_R;
            }
            break;
        case ALS_READ_ONLY_R:
            m_opt3001_state = ALS_READ_ONLY_S;
            break;

        case ALS_INTERRUPT_CLEAR_S:
            if(i2c_use_mode == 0)
            {
                spi_xfer_done = false;
                opt3001_i2c_init();
                opt3001_check_intr_reason();
                opt3001_set_clear_intr();
                m_opt3001_state = ALS_INTERRUPT_CLEAR_R;    
            }
            else
            {
                m_opt3001_state = ALS_INTERRUPT_CLEAR_S;            
            }
            break;
            
        case ALS_INTERRUPT_CLEAR_R:
            cfg_opt3001_timers_stop();
            nrf_drv_gpiote_in_event_enable(PIN_DEF_AIN0, true);
            m_opt3001_state = NONE_ALS;
            break;

        case ALS_SLEEP_S:
            break;
            
        case ALS_SLEEP_R:
            break;

        case EXIT_ALS:
            break;

        default:
            break;
    }

}

/** 
* Initialise OPT3001 light sensor. 
*/ 
void cfg_opt3001_als_init(void)
{
    uint16_t als_intr_lux = 0;
//    uint16_t als_low_int = 0;
    uint16_t low_int, high_int;
    
    opt3001_i2c_init();

    opt3001_get_manufacturer_id();
    opt3001_get_device_id();

    opt3001_get_reg_result();
    opt3001_get_configuration();

    opt3001_set_configuration(DEFAULT_CONFIG_SET);
    opt3001_set_range(OPT3001_RANGE_AUTOMATIC_FULL_SCALE, 0); 
    
    als_intr_lux = ((m_module_parameter.ambient_light_sensor_interrupt_value << 12) | 0xfff);
    if(opt3001_get_enable_low())
    {
        opt3001_set_low_intr(als_intr_lux);
    }
    if(opt3001_get_enable_high())
    {
        opt3001_set_high_intr(als_intr_lux);
    }
    if(CPRINTLOG_ALS)
    {
        opt3001_get_low_intr(&low_int);
        opt3001_get_high_intr(&high_int);
    }
}

void cfg_opt3001_timer_create()
{
    uint32_t err_code;

    // Create timers.
    err_code = app_timer_create(&m_opt3001_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                opt3001_state_handler);
    APP_ERROR_CHECK(err_code);
}

void cfg_opt3001_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_opt3001_timer_id, APP_TIMER_TICKS(100), NULL);
    APP_ERROR_CHECK(err_code);
}
void cfg_opt3001_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code = app_timer_stop(m_opt3001_timer_id);
    APP_ERROR_CHECK(err_code);
}

