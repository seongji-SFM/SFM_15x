/* Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_srv_cgms_meas Continuous Glucose Monitoring Service Measurement
 * @{
 * @ingroup ble_cgms
 * @brief Continuous Glucose Monitoring Service Measurement module.
 *
 * @details This module implements parts of the Continuous Glucose Monitoring that relate to the
 *          Measurement characteristic. Events are propagated to this module from @ref ble_cgms
 *          using @ref cgms_meas_on_write.
 *
 */


#ifndef __MAIN_APP_SIMPLE_H__
#define __MAIN_APP_SIMPLE_H__
#include <stdbool.h>
#include <stdint.h>
#include "cfg_config_app.h"

#define MAIN_APP_BLE_BEACON_INTAERVAL_MS 1000
#define MAIN_APP_GPS_ACQUIRE_TRACKING_TIME_SEC 90
#define MAIN_APP_GPS_OPERATION_MODE 1  //0 : smart mode, 1 : manual mode
#define MAIN_APP_WIFI_SCAN_TYPE CWIFI_SCAN_TYPE_ACTIVE
#define MAIN_APP_WAKEUP_TIMER_INTERVAL_SEC (60*10) //0 is disable

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // NRF_BLE_CFG_MAIN_H__

/** @} */

