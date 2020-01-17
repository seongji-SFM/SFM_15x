#ifndef __CFG_CONFIG_DEFINEDS_H__
#define __CFG_CONFIG_DEFINEDS_H__

/******************************************************
module info
*******************************************************/
#define CDEV_MODEL_NAME "SRM200"   //MODEL NAME SIZE IS 6BYTE
/**********************
module defines (CDEV_)
SFM20R : MODEL_NAME to "SFM20R"
         MODULE_TYPE to CDEV_MODULE_SFM20R
SFM60R : MODEL_NAME to "SFM60R"
         MODULE_TYPE to CDEV_MODULE_SFM60R
SRM200A: MODEL_NAME to "SRM200"
         MODULE_TYPE to CDEV_MODULE_SRM200
***********************/

/******************************************************
module feature (type)
*******************************************************/
#define CDEV_MODULE_SFM20R                      (1)
#define CDEV_MODULE_SFM60R                      (2)
#define CDEV_MODULE_SRM200                      (3)

#define CDEV_MODULE_TYPE                        CDEV_MODULE_SRM200

/******************************************************
version information
*******************************************************/
#define CDEV_SW_VER_MAJOR "3"       // 1byte
#define CDEV_SW_VER_MINOR "00"      // 2byte
#define CDEV_FS_VER 0x0030          //module_parameter_t version
#define CDEV_BL_VER "3"             // 1byte

/******************************************************
board feature
*******************************************************/
#define CDEV_BOARD_EVB                           (1)
#define CDEV_BOARD_IHERE                         (2)
#define CDEV_BOARD_TYPE                          CDEV_BOARD_EVB  //REPLACE_DEVICE_DEFINE_HERE

/******************************************************
wisol feature
*******************************************************/
//#define FEATURE_WISOL_DEVICE //move to preprocessor definitions
//#define FEATURE_WISOL_BOOTLOADER //move to preprocessor definitions
//#define FEATURE_WISOL_APP //move to preprocessor definitions

/******************************************************
supported funcion feature
*******************************************************/
#if (CDEV_BOARD_TYPE == CDEV_BOARD_EVB)
#define FEATURE_CFG_CHECK_BOOTSTRAP_PIN  //support boot strap mode
#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
#define USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN0        PIN_DEF_I2CS_I2C0_SCL_DBG
#define USR_MODULE_PIN_DEF_BOOTSTRAP_MAIN1        PIN_DEF_I2CS_I2C0_SDA_DBG
#define USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG0  PIN_DEF_STATE0 //depend on FEATURE_CFG_CHECK_BOOTSTRAP_PIN
#define USR_MODULE_PIN_DEF_BOOTSTRAP_SUB_CONFIG1  PIN_DEF_WKUP   //depend on FEATURE_CFG_CHECK_BOOTSTRAP_PIN
#endif
#endif

#define FEATURE_CFG_CHECK_NV_BOOT_MODE  //support bootmode (m_module_parameter)
#define FEATURE_CFG_DEBUG_PRINT_OUT //debug print out
#ifndef FEATURE_WISOL_BOOTLOADER
#define FEATURE_CFG_DEBUG_OUT_TO_TBC //depend on FEATURE_CFG_DEBUG_PRINT_OUT
#endif
#define FEATURE_CFG_BYPASS_CONTROL
#define FEATURE_CFG_RTT_MODULE_CONTROL //RTT over TBC(twis board control), Test mode control via RTT
#define FEATURE_CFG_USES_ADC_READ_BATTERY_LEVEL  //adc for battery read
#define FEATURE_CFG_BLE_UART_CONTROL
#define FEATURE_CFG_ACC_REPORT  //depend on FEATURE_CFG_BLE_UART
#define FEATURE_CFG_USE_I2C0_DBG_PIN
#define FEATURE_CFG_FULL_TEST_CMD_WITH_BLE_NUS

/******************************************************
WORKAROUND feature
*******************************************************/
#define FEATURE_WORKAROUND_NFC_CRASH
#define FEATURE_WORKAROUND_I2C_MASTER_SDA_LOW

/******************************************************
DFU feature
*******************************************************/
#define FEATURE_DFU_BOTH_BLE_AND_SEIRAL  //support DFU ble and serial (support "development/SFM/tools/serial_downloader")
#define FEATURE_DFU_SEIRAL_SPEED_UP  //support seiral DFU baudrate 115200


/******************************************************
Common Macro
*******************************************************/
#if 0  //keil
#define CFG_GET_LINK_REGISTER(LR_REG_VAL) __asm { MOV LR_REG_VAL, __return_address()}
#elif 1 //gcc
#define CFG_GET_LINK_REGISTER(LR_REG_VAL) {__ASM volatile ("MOV %0, LR\n" : "=r" (LR_REG_VAL) );}
#else
#define CFG_GET_LINK_REGISTER(LR_REG_VAL)
#endif

/******************************************************
Common Defines
*******************************************************/
#define CFG_ONE_DAY_SEC (60*60*24)


/******************************************************
ble feature
*******************************************************/

/******************************************************
funcion feature
*******************************************************/

//#define CDEV_RTC2_DATE_TIME_CLOCK
/******************************************************/

/******************************************************/

/******************************************************
test feature
*******************************************************/

/******************************************************/


/******************************************************/

#include "cfg_config_module.h"
#include "cfg_config_board.h"
#if defined(FEATURE_WISOL_DEVICE ) && defined(FEATURE_WISOL_APP)
#include "cfg_config_redefine.h"
#endif
#endif // __CFG_CONFIG_DEFINEDS_H__
