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
 * @brief control board device control.
 *
 * This file contains the control wifi modle.
 */


#ifndef __CFG_CONFIG_BOARD_REDEFINE_H__
#define __CFG_CONFIG_BOARD_REDEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#if((CDEV_MODULE_TYPE == CDEV_MODULE_SRM200) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB))  //CDEV_SIGFOX_MONARCH_MODULE
#undef SIGFOX_SCAN_RC_MODE_DEFAULT
#define SIGFOX_SCAN_RC_MODE_DEFAULT     RC_SCAN_ONLY_SEND_WHEN_CONFIRMED
#endif

#if (((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB))
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#elif  (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) 
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#else
#define ADJUST_BATTERY_VALUE    50
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) (((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) * 5/3)
#endif

#ifdef __cplusplus
}
#endif
#endif // __CFG_CONFIG_BOARD_REDEFINE_H__
