1. Main Scenario in main_scheduler_state_machine()
GPS -> WIFI -> Sigfox Scan -> Sigfox send -> Sleep

2. Peripherals in board_peripherals_handler()
Read Internal Accelerometer
Read GPIOs Level

3. Support DFU (Ble Fota) - cfg_ble_advertising_start_stop_req(true);
ref cfg_ble_services_init() 1st param

4. Support NUS (Nordic Uart Service) - cfg_ble_advertising_start_stop_req(true);
ref cfg_ble_services_init() 2st, 3th param (FEATURE_CFG_BLE_UART_CONTROL)

5. Support PC tool Connect with RTT Viewer - ref FEATURE_CFG_RTT_MODULE_CONTROL

6. Support Bootmode - ref FEATURE_CFG_CHECK_NV_BOOT_MODE

*Available Setting Items (in module_parameter_t)
boot_mode -> It can be changed by RTT Viewer CMD. (RTT CMD eg. CM1) 
log_mask -> It can be changed by PC Tool CMD. (PC Tool CMD eg. <SC>0C040316DB7DB7)
sigfox_recv_en -> enable recv data with sigfox (PC Tool CMD eg. <SC>0C040B00000001 -> enable)
sigfox_snek_testmode_enable -> it use sigfox public key (PC Tool CMD eg. <SC>0C040F00000001 -> enable)
sigfox_RC_number -> sigfox RC zone Setting for monarch (PC Tool CMD eg. <SC>0C041800000001 -> RC1)
sigfox_scan_rc_mode -> ref rc_scan_type_e with monarch (PC Tool CMD eg. <SC>0C041900000000 -> disble)

*RTT Viewer Command
CMx -> mode change (eg. CM1)
CR -> Reset
CF -> Setting Value to Default

*PC Tool CMDs Protocal -> see Host_FT20_Protocal.txt

*Available GPIOs
1. AIN0 (It can also be used as ADC)
2. AIN1 (It can also be used as ADC)
3. WKUP
4. STATE0
5. I2C0_SCL_DBG (It can also be used as I2C master or I2C slave)
6. I2C0_SDA_DBG (It can also be used as I2C master or I2C slave)
7. NFC1 (It can also be used as NFC)
8. NFC2 (It can also be used as NFC)

* ADC -> Ref USR_MODULE_DEF_BATTERY_ADC_INPUT
* I2C master with I2C0 DBG -> Ref FEATURE_CFG_USE_I2C0_DBG_PIN
* I2C slave with I2C0 DBG -> Ref FEATURE_CFG_USE_I2CM_2nd (need add TWIS_ENABLED=0;TWIS1_ENABLED=0;TWI1_ENABLED=1 in predefine)
* NFC -> Ref USR_MODULE_FUNCTION_USE_NFC (remove CONFIG_NFCT_PINS_AS_GPIOS in predefine)