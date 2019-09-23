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


#ifndef __CFG_CONFIG_BOARD_H__
#define __CFG_CONFIG_BOARD_H__

#if ((((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    || ((CDEV_MODULE_TYPE == CDEV_MODULE_SRM200) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    )
#define USR_MODULE_GPIO_DEF_WAKEUP_KEY PIN_DEF_WKUP
#define USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON  //depend on USR_MODULE_GPIO_DEF_WAKEUP_KEY
#endif

#if ((((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) \
    )
#define USR_MODULE_GPIO_DEF_BLE_LED PIN_DEF_STATE0
#define USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON (1)   // 0 or 1
#endif

#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE)
#define USR_MODULE_GPIO_DEF_LED_2 PIN_DEF_WKUP
#define USR_MODULE_GPIO_DEF_LED_2_HIGH_TO_ON (1)   // 0 or 1
#endif

#if (((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB))
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL PIN_DEF_AIN1  //close to low
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON  //depend on USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_ACTIVE_LEVEL 0  //depend on USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
#elif (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE)
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL PIN_DEF_AIN0  //close to high
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON  //depend on USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
#define USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_ACTIVE_LEVEL 1  //depend on USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL
#endif

#if 0  //operation is OK, but not used.
#define USR_MODULE_GPIO_DEF_BUTTON PIN_DEF_I2CS_I2C0_SCL_DBG
#define USR_MODULE_GPIO_DEF_BUTTON_ACTIVE_LEVEL 1  //depend on USR_MODULE_GPIO_DEF_BUTTON
#define USR_MODULE_GPIO_DEF_BUTTON_POWER_OFF 1  //depend on USR_MODULE_GPIO_DEF_BUTTON
#endif

#if (((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB))
#define USR_MODULE_DEF_BATTERY_ADC_INPUT NRF_SAADC_INPUT_AIN0
#elif (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE)
#define USR_MODULE_DEF_BATTERY_ADC_INPUT NRF_SAADC_INPUT_AIN1
#endif

/******************************************************
Board-dependent functions features.
*******************************************************/
#if !defined (CONFIG_NFCT_PINS_AS_GPIOS)  //it define is preprocessor definitions
#define USR_MODULE_FUNCTION_USE_NFC  //NFC Tag mode for Read IDs
#if ((((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) \
    )
#define USR_MODULE_FUNCTION_USE_NFC_POWERON  //depend on USR_MODULE_FUNCTION_USE_NFC
#endif
#endif

#if ((((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    || ((CDEV_MODULE_TYPE == CDEV_MODULE_SRM200) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    || (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE) \
    )
#define BD_FEATURE_POWEROFF_AT_FIRSTTIME  //power off at first power applied.
#if ((CDEV_MODULE_TYPE == CDEV_MODULE_SRM200) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB))
#define BD_FEATURE_POWEROFF_AT_FIRSTTIME_WHEN_I2CDBG_NOT_CONNCETED
#endif
#endif

/******************************************************
external sensor feature
*******************************************************/
#if ((((CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SFM60R)) && (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)) \
    )
#define CDEV_TEMPERATURE_SENSOR_TMP102
#endif
//#define CDEV_TEMPERATURE_SENSOR_TMP108
//#define CDEV_AMBIENT_LIGHT_SENSOR
//#define CDEV_EXTERNAL_I2C_SENSOR_INT_PIN PIN_DEF_AIN0

/******************************************************
datalayout feature
*******************************************************/
#define LEGACY_SIGFOX_DATA_FORMAT          (1)
#define NEW_SIGFOX_DATA_FORMAT_V2          (2)
#define CFG_UPLOAD_DATA_FORMAT_TYPE            (LEGACY_SIGFOX_DATA_FORMAT)

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
