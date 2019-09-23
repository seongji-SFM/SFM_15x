/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
*
* The information contained herein is property of WISOL Cor.
* Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
*
* This heading must NOT be removed from
* the file.
*
*/
#ifndef __CFG_ADC_BATTERY_CHECK_H__
#define __CFG_ADC_BATTERY_CHECK_H__
#include <stdbool.h>
#include <stdint.h>
#include "cfg_config_defines.h"
#include "nrf_saadc.h"
#ifdef __cplusplus
extern "C" {
#endif

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(50)  /**< Battery level measurement interval (ticks). */
#define BATTERY_LEVEL_AVG_CNT 3

#define BATTERY_ADC_INPUT_MIN   (2500)
#define BATTERY_ADC_INPUT_MAX   (5200)
#define BATTERY_ADC_WARNING_LVL (3550)
#define BATTERY_ADC_CUT_OFF_LVL (3450)

extern uint8_t   batt_avg_report_volts;
extern uint8_t   batt_avg_report_percent;

void cfg_battery_check_init(nrf_saadc_input_t adc_input);
void cfg_battery_check_start(void);
uint16_t get_avg_batt_lvl_in_milli_volts(void);

#ifdef __cplusplus
}
#endif
#endif // __CFG_SPECIAL_BOOT_MODE_H__

