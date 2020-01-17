/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
*/
#include "cfg_adc_battery_check.h"

#include <stdint.h>
#include <string.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_timer.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"

uint8_t   batt_avg_report_volts;
uint8_t   batt_avg_report_percent;

#if defined(FEATURE_CFG_USES_ADC_READ_BATTERY_LEVEL) && defined(USR_MODULE_DEF_BATTERY_ADC_INPUT)
uint16_t total_batt_lvl_in_milli_volts;
static nrf_saadc_input_t m_cfg_adc_batt_input;
static bool m_cfg_adc_batt_init_flag = false;

__WEAK uint16_t batt_lvl_array[][2] = 
{
    {100, 4000},
    {90, 3910},
    {80, 3840},
    {70, 3800},
    {60, 3760},
    {50, 3720},
    {40, 3680},
    {30, 3640},        
    {20, 3600},
    {10, BATTERY_ADC_WARNING_LVL},
    {5, BATTERY_ADC_CUT_OFF_LVL}
};
__WEAK uint16_t batt_lvl_array_cnt = (sizeof(batt_lvl_array)/4);

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200                                         /**< Value in millivolts for voltage used as reference in ADC conversion on NRF51. */
#define ADC_INPUT_PRESCALER               3                                            /**< Input prescaler for ADC convestion on NRF51. */
#define ADC_RES_10BIT                     1024                                         /**< Maximum digital value for 10-bit ADC conversion. */

static uint8_t make_batt_lvl_report_data(uint16_t batt_lvl)
{
    int i;
    int batt_percent = 0;
    for(i=0; i < batt_lvl_array_cnt; i++)
    {
        if(batt_lvl >= batt_lvl_array[i][1])
        {            
            batt_percent = batt_lvl_array[i][0];
            break;
        }
    }
    return batt_percent;
}
APP_TIMER_DEF(m_battery_timer_id);                                               /**< Battery measurement timer. */

//static ble_bas_t                        m_bas;                                   /**< Structure used to identify the battery service. */
static nrf_saadc_value_t adc_buf[2];

uint16_t get_avg_batt_lvl_in_milli_volts(void);

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    #ifdef ADC_PRESENT
    nrf_drv_adc_sample();
    #else // SAADC_PRESENT
    uint32_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    #endif // ADC_PRESENT
}

static void cfg_bas_timer_create(void)
{
    uint32_t   err_code;

    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void cfg_bas_timer_start(void)
{
    uint32_t    err_code;

    batt_avg_report_volts = 0;
    batt_avg_report_percent = 0;
    total_batt_lvl_in_milli_volts = 0;
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void un_adc_configure(void)
{
    nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();
}

 /**@brief Function for handling the ADC interrupt.
  *
  * @details  This function will fetch the conversion result from the ADC, convert the value into
  *           percentage and send it to peer.
  */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    static uint8_t  check_count = 0;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t   adc_result;
        uint16_t            batt_lvl_in_milli_volts;
        uint32_t            err_code;

        adc_result = p_event->data.done.p_buffer[0];

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) + ADJUST_BATTERY_VALUE;
        if(check_count++ < BATTERY_LEVEL_AVG_CNT)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
            APP_ERROR_CHECK(err_code);

            total_batt_lvl_in_milli_volts += batt_lvl_in_milli_volts;
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            cPrintLog(CDBG_EXT_SEN_DBG, "BATTERY cur:%d cnt:%d\n", batt_lvl_in_milli_volts, check_count);
        }
        else
        {
            batt_avg_report_volts = ((get_avg_batt_lvl_in_milli_volts()) / 100);
            batt_avg_report_percent = make_batt_lvl_report_data((get_avg_batt_lvl_in_milli_volts()));

            check_count = 0;
            un_adc_configure();
            cPrintLog(CDBG_EXT_SEN_INFO, "BATTERY avg:%d, report_volts:%d report_per:%d\n", get_avg_batt_lvl_in_milli_volts(), batt_avg_report_volts, batt_avg_report_percent);
        }
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    #ifdef ADC_PRESENT
    ret_code_t err_code = nrf_drv_adc_init(NULL, adc_event_handler);
    APP_ERROR_CHECK(err_code);

    static nrf_drv_adc_channel_t channel =
        NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_DISABLED);
    // channel.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&channel);

    err_code = nrf_drv_adc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);
    #else //  SAADC_PRESENT
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(m_cfg_adc_batt_input);

    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
    #endif //ADC_PRESENT
}

void cfg_battery_check_init(nrf_saadc_input_t adc_input)
{
    m_cfg_adc_batt_input = adc_input;
    cfg_bas_timer_create();
    m_cfg_adc_batt_init_flag = true;
}

void cfg_battery_check_start(void)
{
    if(!m_cfg_adc_batt_init_flag)return;
    adc_configure();
    cfg_bas_timer_start();
}

uint16_t get_avg_batt_lvl_in_milli_volts(void)
{
    return (total_batt_lvl_in_milli_volts/BATTERY_LEVEL_AVG_CNT);
}
#else
void cfg_battery_check_init(nrf_saadc_input_t adc_input){ UNUSED_PARAMETER(adc_input);}
void cfg_battery_check_start(void){}
uint16_t get_avg_batt_lvl_in_milli_volts(void){}
#endif
