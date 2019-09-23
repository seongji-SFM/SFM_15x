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


#ifndef __CFG_CONFIG_MODULE_H__
#define __CFG_CONFIG_MODULE_H__

/******************************************************
module function feature
*******************************************************/
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SFM20R) || (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)
#define CDEV_WIFI_MODULE    //ESP8285
#endif
#define CDEV_GPS_MODULE     //UBX-G8020
#if (CDEV_MODULE_TYPE == CDEV_MODULE_SRM200)
#define CDEV_SIGFOX_MONARCH_MODULE
#else
#define CDEV_SIGFOX_MODULE  //onsemi AX-SFEU
#endif
#define CDEV_ACC_MODULE    //Accelerometer (BMA250 or BMA253)
#define CDEV_BLE_ADVERTISING_ENABLE
#define CDEV_BLE_SCAN_ENABLE

/******************************************************
define of interrupt resource
*******************************************************/
//interrupt source 0 : i2c master, 1 : i2c master, 2 spi (wifi and gsp can not be used at same time.)
//I2C Master
#define GSEN_TWI_INSTANCE           0   //used by Accelerometer(bma250) and external sensor

//I2C INSTANCE
#define TBC_TWIS_INSTANCE           1  /**< I2C instance index for  twis board control ref TWIS_ENABLED TWIS1_ENABLED*/

//SPI INSTANCE
#define WIFI_SPI_INSTANCE           2  /**< SPI instance index. */
#define GPS_SPI_INSTANCE            2  /**< SPI instance index. */

/******************************************************
define of pin resource
*******************************************************/
#define PIN_DEF_2ND_POW_EN          26
#define PIN_DEF_WIFI_INT            4  //for download mode DL_EN/INT_WIFI

#define PIN_DEF_SIGFOX_PWR_EN       24
#define PIN_DEF_SIGFOX_RESET        23
#define PIN_DEF_SIGFOX_UART_TX      25
#define PIN_DEF_SIGFOX_UART_RX      27

#define PIN_DEF_WIFI_SPI_MISO       8
#define PIN_DEF_WIFI_SPI_MOSI       6
#define PIN_DEF_WIFI_SPI_CLK        7
#define PIN_DEF_WIFI_SPI_CS         5
#define PIN_DEF_WIFI_PWR_EN         13
#define PIN_DEF_WIFI_RESET          17

#define PIN_DEF_GPS_SPI_MISO        28
#define PIN_DEF_GPS_SPI_MOSI        29
#define PIN_DEF_GPS_SPI_SCK         30
#define PIN_DEF_GPS_SPI_CS          31
#define PIN_DEF_GPS_PWR_EN          19
#define PIN_DEF_GPS_RESET           22

#define PIN_DEF_I2CM_I2C1_SCL_BLE   15  //i2c master pin in the module
#define PIN_DEF_I2CM_I2C1_SDA_BLE   16  //i2c master pin in the module
#define PIN_DEF_INT1_ACC            11  //it is acvice low

#define PIN_DEF_I2CS_I2C0_SCL_DBG   12  //i2c slave pin in the module
#define PIN_DEF_I2CS_I2C0_SDA_DBG   14  //i2c slave pin in the module

#if defined (CONFIG_NFCT_PINS_AS_GPIOS)  //it define is preprocessor definitions
#define PIN_DEF_NFC1                9
#define PIN_DEF_NFC2                10
#endif

#define PIN_DEF_WKUP                20
#define PIN_DEF_STATE0              18
#define PIN_DEF_AIN0                2 
#define PIN_DEF_AIN1                3 

#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
#define PIN_DEF_DTM_RX              PIN_DEF_AIN0 //used source_direct_test_mode
#define PIN_DEF_DTM_TX              PIN_DEF_AIN1 //used source_direct_test_mode
#else
#define PIN_DEF_DTM_RX              PIN_DEF_I2CS_I2C0_SCL_DBG //used source_direct_test_mode
#define PIN_DEF_DTM_TX              PIN_DEF_I2CS_I2C0_SDA_DBG //used source_direct_test_mode
#endif

#endif // __WBOARD_CONFIG_DEF__
