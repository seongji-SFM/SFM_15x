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


#ifndef __CFG_CONFIG_REDEFINE_H__
#define __CFG_CONFIG_REDEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#undef USR_MODULE_DEF_BATTERY_ADC_INPUT  //not use battery check
#undef USR_MODULE_GPIO_DEF_BLE_LED       //not use status led
#undef FEATURE_CFG_USE_I2C0_DBG_PIN      //not use i2c dbg
#undef USR_MODULE_FUNCTION_USE_NFC       //not use NFC. it must use predefine CONFIG_NFCT_PINS_AS_GPIOS in project files
#undef FEATURE_CFG_USE_I2CM_2nd
#undef USR_MODULE_DEF_BATTERY_ADC_INPUT  //not use ADC for Battery Level Check

#define CDEV_DISABLE_ACC_MODULE_LIB    //Exclude Accelerometer

#ifdef __cplusplus
}
#endif
#endif // __CFG_CONFIG_REDEFINE_H__