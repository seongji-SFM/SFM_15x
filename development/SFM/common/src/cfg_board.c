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
 * @brief tracking Sample Application cfg_board.c file.
 *
 * This file contains the source code for an tracking sample application.
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
#include "nrf_pwr_mgmt.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_gps_module.h"
#include "cfg_twis_board_control.h"
#include "cfg_external_sense_gpio.h"
#include "cfg_sigfox_module.h"
#include "main_demoApp.h"
#include "cfg_nvm_ctrl.h"
#include "cfg_ble_ctrl.h"
#include "cfg_special_boot_mode.h"
#include "cfg_board_update_wifi_tx_power_tables.h"
#include "cfg_nRF52_peripherals.h"

const char m_cfg_sw_ver[4] = {CDEV_SW_VER_MAJOR CDEV_SW_VER_MINOR};
const char m_cfg_model_name[16] = 
{
#if (CDEV_BOARD_TYPE == CDEV_BOARD_IHERE)
    "iHere"
#else
    CDEV_MODEL_NAME
#endif
};
const char m_cfg_board_type = CDEV_BOARD_TYPE;
const char m_cfg_build_date[] = __DATE__;
const char m_cfg_build_time[] = __TIME__;

bool m_cfg_sw_reset_detected = false;
bool m_cfg_debug_interface_wake_up_detected = false;
bool m_cfg_available_bootloader_detected = false;
bool m_cfg_NFC_wake_up_detected = false;
bool m_cfg_GPIO_wake_up_detected = false;
bool m_cfg_i2c_master_init_flag = false;
#ifdef FEATURE_WORKAROUND_I2C_MASTER_SDA_LOW
static bool m_cfg_i2c_master_init_first_flag = false;
#endif

int m_cfg_comm_pwr_mask = 0;
const nrf_drv_spi_config_t m_spi_config_default = NRF_DRV_SPI_DEFAULT_CONFIG;
cfg_board_shutdown_peripherals_func m_cfg_shutdown_peripherals_func = NULL;

/*****************************************************************************************/
//It is a variable that can not be located in main.c(It is referred to in several places.)
/*****************************************************************************************/
uint8_t  magnet_status;  //0:open, 1:close
/*****************************************************************************************/
#ifdef CDEV_RTC2_DATE_TIME_CLOCK
#define MAX_RTC2_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. ref MAX_RTC_TASKS_DELAY*/
#define RTC2_TIMER_CONFIG_IRQ_PRIORITY 7  //ref APP_TIMER_CONFIG_IRQ_PRIORITY
#define DATE_TIME_CLOCK_DEFAULT (1483196400+32400)  //2017.01.01-00:00:00 based linux timestamp - max is 2038-01-19 03:14:07

static uint32_t date_time_clock_timestamp;
static bool date_time_clock_start_flag = false;

void RTC2_IRQHandler(void)
{
    cPrintLog(CDBG_COMMON_LOG, "Rtc2 IRQ occurred\n");

    date_time_clock_timestamp += 2097152;  //PRESCALER 4095 =  2097152 sec - about 582.542 hours //PRESCALER 0 is 512 sec
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->EVENTS_COMPARE[1] = 0;
    NRF_RTC2->EVENTS_COMPARE[2] = 0;
    NRF_RTC2->EVENTS_COMPARE[3] = 0;
    NRF_RTC2->EVENTS_TICK       = 0;
    NRF_RTC2->EVENTS_OVRFLW     = 0;
}

void RTC2_date_time_clock_init_N_start(void)  //date_time
{
    NRF_RTC2->PRESCALER = 4095;  //2097152 sec  //0 is 512 sec
    NVIC_SetPriority(RTC2_IRQn, RTC2_TIMER_CONFIG_IRQ_PRIORITY);

    NRF_RTC2->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(RTC2_IRQn);
    NVIC_EnableIRQ(RTC2_IRQn);

    date_time_clock_timestamp = DATE_TIME_CLOCK_DEFAULT;
    NRF_RTC2->TASKS_START = 1;
    nrf_delay_us(MAX_RTC2_TASKS_DELAY);
    date_time_clock_start_flag = true;
}

uint32_t date_time_get_timestamp(void)  //date_time
{
    uint32_t RTC2_counter;
    uint32_t timestamp = 0;

    if(date_time_clock_start_flag)
    {
        RTC2_counter = NRF_RTC2->COUNTER;
        timestamp = (date_time_clock_timestamp + (RTC2_counter / 8));
    }
    return timestamp;
}

void date_time_set_timestamp(uint32_t timestamp)
{
    uint32_t RTC2_counter;

    RTC2_counter = NRF_RTC2->COUNTER;
    if(timestamp < (RTC2_counter / 8))
    {
        date_time_clock_timestamp = 0;
    }
    else
    {
        date_time_clock_timestamp = timestamp - (RTC2_counter / 8);
    }
}

void date_time_get_current_time(cfg_date_time_t *tm)
{
  uint32_t seconds, minutes, hours, days, year, month;
  uint32_t dayOfWeek;
  seconds = date_time_get_timestamp();

  /* calculate minutes */
  minutes  = seconds / 60;
  seconds -= minutes * 60;
  /* calculate hours */
  hours    = minutes / 60;
  minutes -= hours   * 60;
  /* calculate days */
  days     = hours   / 24;
  hours   -= days    * 24;

  /* Unix time starts in 1970 on a Thursday */
  year      = 1970;
  dayOfWeek = 4;

  while(1)
  {
    bool     leapYear   = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    uint16_t daysInYear = leapYear ? 366 : 365;
    if (days >= daysInYear)
    {
      dayOfWeek += leapYear ? 2 : 1;
      days      -= daysInYear;
      if (dayOfWeek >= 7)
        dayOfWeek -= 7;
      ++year;
    }
    else
    {
      dayOfWeek  += days;
      dayOfWeek  %= 7;

      /* calculate the month and day */
      static const uint8_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      for(month = 0; month < 12; ++month)
      {
        uint8_t dim = daysInMonth[month];

        /* add a day to feburary if this is a leap year */
        if (month == 1 && leapYear)
          ++dim;

        if (days >= dim)
          days -= dim;
        else
          break;
      }
      break;
    }
  }

  tm->seconds  = seconds;
  tm->minutes  = minutes;
  tm->hours = hours;
  tm->day = days + 1;
  tm->month = month + 1;
  tm->year = year;
  tm->dayOfWeek = dayOfWeek;
}
#endif

void cfg_board_common_power_control(module_comm_pwr_resource_e resource, bool bOn)
{
    if(bOn)
    {
        if(m_cfg_comm_pwr_mask == 0)
        {
#ifdef PIN_DEF_2ND_POW_EN
            nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 1);
#endif
        }
        m_cfg_comm_pwr_mask = (m_cfg_comm_pwr_mask | (0x01<<resource));
    }
    else
    {
        m_cfg_comm_pwr_mask = (m_cfg_comm_pwr_mask & ~(0x01<<resource));
        if(m_cfg_comm_pwr_mask == 0)
        {
#ifdef PIN_DEF_2ND_POW_EN
            nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
            nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
#endif
        }
    }
}

static bool old_ble_led_status = false;
bool cfg_get_ble_led_status(void)
{
    return old_ble_led_status;
}

bool cfg_ble_led_control(bool bOn)
{    
    bool ret = 0;
#ifdef USR_MODULE_GPIO_DEF_BLE_LED
    static bool bGpioInit = false;
    if(!bGpioInit)
    {
        nrf_gpio_cfg_output(USR_MODULE_GPIO_DEF_BLE_LED);
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, !USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON);
        bGpioInit = true;
    }
    ret = old_ble_led_status;
    old_ble_led_status = bOn;
    if(bOn)
    {
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON);
    }
    else
    {
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, !USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON);
    }
#endif
    return ret;
}

static bool old_led_2_status = false;
bool cfg_get_led_2_status(void)
{
    return old_led_2_status;
}
bool cfg_led_2_control(bool bOn)
{    
    bool ret = 0;
#ifdef USR_MODULE_GPIO_DEF_LED_2
    static bool bGpioInit = false;
    if(!bGpioInit)
    {
        nrf_gpio_cfg_output(USR_MODULE_GPIO_DEF_LED_2);
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_LED_2, !USR_MODULE_GPIO_DEF_LED_2_HIGH_TO_ON);
        bGpioInit = true;
    }
    ret = old_led_2_status;
    old_led_2_status = bOn;
    if(bOn)
    {
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_LED_2, USR_MODULE_GPIO_DEF_LED_2_HIGH_TO_ON);
    }
    else
    {
        nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_LED_2, !USR_MODULE_GPIO_DEF_LED_2_HIGH_TO_ON);
    }
#endif
    return ret;
}

const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(GSEN_TWI_INSTANCE);
/* Indicates if operation on TWI has ended. */
volatile bool spi_xfer_done = false;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch (p_event->xfer_desc.type )
            {
                case NRF_DRV_TWI_XFER_TX :
                    spi_xfer_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX: 
                    spi_xfer_done = true; 
                    break; 
                case NRF_DRV_TWI_XFER_RX: 
                    spi_xfer_done = true; 
                    break; 
                case NRF_DRV_TWI_XFER_TXRX: 
                    spi_xfer_done = true; 
                    break; 
                default: 
                    break; 
            }
            break;
        default:
            break;
    }
}

/**@brief Function for initializing I2C in accelerometer.
 *                         
 */
void cfg_i2c_master_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = PIN_DEF_I2CM_I2C1_SCL_BLE,
       .sda                = PIN_DEF_I2CM_I2C1_SDA_BLE,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

#ifdef FEATURE_WORKAROUND_I2C_MASTER_SDA_LOW
    if(!m_cfg_i2c_master_init_first_flag)
    {
        int CLK_cnt = 8;
        m_cfg_i2c_master_init_first_flag = true;
        nrf_gpio_cfg_input(PIN_DEF_I2CM_I2C1_SDA_BLE, NRF_GPIO_PIN_NOPULL);
        nrf_delay_us(100);
        if(!nrf_gpio_pin_read(PIN_DEF_I2CM_I2C1_SDA_BLE))
        {            
            nrf_gpio_cfg_output(PIN_DEF_I2CM_I2C1_SCL_BLE);
            cPrintLog(CDBG_MAIN_LOG, "=== warning SDA is Low! make CLK, stop ===\n", CLK_cnt);
            do
            {
                nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 0);
                nrf_delay_us(100);
                nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 1);
                nrf_delay_us(100);
            }while(--CLK_cnt);
            //make stop condition
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 0);
            nrf_delay_us(50);
            nrf_gpio_cfg_output(PIN_DEF_I2CM_I2C1_SDA_BLE);
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SDA_BLE, 0);
            nrf_delay_us(50);
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 1);
            nrf_delay_us(100);
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 0);
            nrf_delay_us(100);
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SCL_BLE, 1);
            nrf_delay_us(50);
            nrf_gpio_pin_write(PIN_DEF_I2CM_I2C1_SDA_BLE, 1);
            nrf_delay_ms(100);
        }
    }
#endif
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    m_cfg_i2c_master_init_flag = true;
}
/**@brief Function for uninitializing I2C in accelerometer.
 *                         
 */

void cfg_i2c_master_uninit(void)
{
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);
    m_cfg_i2c_master_init_flag = false;
}

uint32_t cfg_i2c_master_send_General_Call_Reset(void)
{
    uint32_t timeout;
    uint8_t array[4];
    
    spi_xfer_done = false;
    timeout = 100000;
    array[0] = 0x06;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, 0, array, 1, false));
    while((!spi_xfer_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    nrf_delay_ms(1);
    return NRF_SUCCESS;
}

static void cfg_board_resource_init(void)
{
}

void cfg_board_reset(void)
{
    NVIC_SystemReset();
}

void cfg_board_check_reset_reason(void)
{
    unsigned int reset_reason = NRF_POWER->RESETREAS;

    cPrintLog(CDBG_MAIN_LOG, "Check Reset Reason:0x%08x\n", reset_reason);

    if(reset_reason & POWER_RESETREAS_RESETPIN_Msk)  //reset pin or power on reset
    {
        cPrintLog(CDBG_MAIN_LOG, "HW_Reset_Det! (POR)\n");
        NRF_POWER->RESETREAS = POWER_RESETREAS_RESETPIN_Msk | POWER_RESETREAS_SREQ_Msk | POWER_RESETREAS_DIF_Msk | POWER_RESETREAS_NFC_Msk | POWER_RESETREAS_OFF_Msk | POWER_RESETREAS_LOCKUP_Msk;
    }
    else
    {
        if(reset_reason & POWER_RESETREAS_SREQ_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "SW_Reset_Det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_SREQ_Msk;
            m_cfg_sw_reset_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_DIF_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "Debug_Interface_Wake_Up\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_DIF_Msk;
            m_cfg_debug_interface_wake_up_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_NFC_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "NFC_Wake_Up_Det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_NFC_Msk;
            m_cfg_NFC_wake_up_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_OFF_Msk || reset_reason == 0)
        {
            cPrintLog(CDBG_MAIN_LOG, "GPIO_Wake_Up_Det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_OFF_Msk;
            m_cfg_GPIO_wake_up_detected = true;
        }

        if(reset_reason & POWER_RESETREAS_LOCKUP_Msk)
        {
            cPrintLog(CDBG_MAIN_LOG, "LOCKUP_Det\n");
            NRF_POWER->RESETREAS = POWER_RESETREAS_LOCKUP_Msk;
        }

    }

}

void cfg_board_check_bootloader(void)
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    if(bootloader_addr != 0xFFFFFFFF)
    {
        cPrintLog(CDBG_BLE_INFO, "bootloader detected:%p\n", (void *)bootloader_addr);
        m_cfg_available_bootloader_detected = true;
    }
    else
    {
        cPrintLog(CDBG_BLE_ERR, "bootloader Not detected!\n");
    }
    cPrintLog(CDBG_MAIN_LOG, "BL ADDR [mbr:%p, uicr:%p]\n", (*(uint32_t *)MBR_BOOTLOADER_ADDR), *MBR_UICR_BOOTLOADER_ADDR);
}

void cfg_board_gpio_set_default_gps(void)
{
#ifdef CDEV_GPS_MODULE
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_GPS_RESET);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_SCK);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_CS);
#endif
}

void cfg_board_gpio_set_default(void)
{
#ifdef CDEV_WIFI_MODULE
    nrf_gpio_cfg_output(PIN_DEF_WIFI_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_WIFI_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_WIFI_RESET);
    nrf_gpio_pin_write(PIN_DEF_WIFI_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_WIFI_INT);

    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_CLK);
    nrf_gpio_cfg_default(PIN_DEF_WIFI_SPI_CS);
#endif

#if 0 // def CDEV_GPS_MODULE  // temp
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_cfg_output(PIN_DEF_GPS_RESET);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);

    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MISO);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_MOSI);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_SCK);
    nrf_gpio_cfg_default(PIN_DEF_GPS_SPI_CS);
#endif

    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_SIGFOX_PWR_EN, 0);
//    nrf_gpio_cfg_output(PIN_DEF_SIGFOX_RESET);
//    nrf_gpio_pin_write(PIN_DEF_SIGFOX_RESET, 0);
//    nrf_gpio_cfg_input(PIN_DEF_SIGFOX_RESET, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_default(PIN_DEF_SIGFOX_RESET);

    nrf_gpio_cfg_default(PIN_DEF_SIGFOX_UART_TX);
    nrf_gpio_cfg_default(PIN_DEF_SIGFOX_UART_RX);
    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
    nrf_gpio_cfg_default(PIN_DEF_I2CS_I2C0_SCL_DBG);
    nrf_gpio_cfg_default(PIN_DEF_I2CS_I2C0_SDA_DBG);

    nrf_gpio_cfg_default(PIN_DEF_I2CM_I2C1_SCL_BLE);
    nrf_gpio_cfg_default(PIN_DEF_I2CS_I2C0_SDA_DBG);
    nrf_gpio_cfg_default(PIN_DEF_INT1_ACC);

    nrf_gpio_cfg_default(PIN_DEF_WKUP);
    nrf_gpio_cfg_default(PIN_DEF_STATE0);

    nrf_gpio_cfg_default(PIN_DEF_AIN0);
    nrf_gpio_cfg_default(PIN_DEF_AIN1);

#ifdef USR_MODULE_GPIO_DEF_BLE_LED 
    nrf_gpio_cfg_output(USR_MODULE_GPIO_DEF_BLE_LED);
    nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, !USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON);
#endif

#ifdef USR_MODULE_GPIO_DEF_LED_2 
    nrf_gpio_cfg_output(USR_MODULE_GPIO_DEF_LED_2);
    nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_LED_2, !USR_MODULE_GPIO_DEF_LED_2_HIGH_TO_ON);
#endif

}

/**@brief Function for doing power management.
 */
void cfg_board_power_manage(void)  //doing power management. (arm sleep)
{
    nrf_pwr_mgmt_run();
}

void cfg_board_indicate_power_down(void)
{
    int i;
    for(i=0;i<10;i++)
    {
        cfg_ble_led_control(((i%2==0)?true:false));
        nrf_delay_ms(100);
    }
}

void cfg_board_prepare_power_down(void)
{
    cPrintLog(CDBG_MAIN_LOG, "prepare power down\n");
    if(!m_cfg_i2c_master_init_flag)
        cfg_i2c_master_init();
    nrf_delay_ms(1);
    cfg_i2c_master_send_General_Call_Reset();
    nrf_delay_ms(1);
#ifdef CDEV_ACC_MODULE
    cfg_bma250_req_suppend_mode();
    nrf_delay_ms(1);
#endif

#if defined(CDEV_TEMPERATURE_SENSOR_TMP102) || defined(CDEV_TEMPERATURE_SENSOR_TMP108)
    tmp102_req_shutdown_mode();
    nrf_delay_ms(1);
#endif

#ifdef CDEV_AMBIENT_LIGHT_SENSOR
    opt3001_set_shutdown();
    nrf_delay_ms(1);
#endif
    if(m_cfg_i2c_master_init_flag)
        cfg_i2c_master_uninit();
    nrf_delay_ms(1);
    
    cfg_board_gpio_set_default();
#ifdef USR_MODULE_FUNCTION_USE_NFC
    cfg_nfc_uninit();
#endif
    nrf_delay_ms(1);

    cfg_board_gpio_set_default_gps(); //gpio relese for gps
    nrf_delay_ms(1);

    nrf_gpio_cfg_default(PIN_DEF_WKUP);

#ifdef CDEV_GPS_MODULE //stop gps backup battery
    cGps_power_control(false, false);  //gps power off
    nrf_delay_ms(1);

    //gps backup battery unuse
    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 1);
    nrf_delay_ms(10);
    nrf_gpio_pin_write(PIN_DEF_GPS_RESET, 0);
    nrf_delay_ms(1);
    nrf_gpio_cfg_output(PIN_DEF_GPS_PWR_EN);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 1);
    nrf_delay_ms(1);
    nrf_gpio_pin_write(PIN_DEF_GPS_PWR_EN, 0);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);

    cfg_board_gpio_set_default_gps(); //gpio relese for gps
#endif

#if defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY) && defined(USR_MODULE_GPIO_DEF_WAKEUP_KEY_POWERON)
    {
        uint32_t pin_number = USR_MODULE_GPIO_DEF_WAKEUP_KEY;
        nrf_gpio_pin_sense_t sense_config = NRF_GPIO_PIN_SENSE_HIGH;
        nrf_gpio_cfg_sense_input(pin_number, NRF_GPIO_PIN_PULLDOWN, sense_config);
        nrf_gpio_cfg_sense_set(pin_number, sense_config);
        nrf_delay_ms(1);
    }
#endif

#if defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL) && defined(USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_POWERON)
    {
        uint32_t pin_number = USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL;
#if USR_MODULE_GPIO_DEF_MAGNETIC_SIGNAL_ACTIVE_LEVEL
        nrf_gpio_pin_sense_t sense_config = NRF_GPIO_PIN_SENSE_HIGH;
#else
        nrf_gpio_pin_sense_t sense_config = NRF_GPIO_PIN_SENSE_LOW;
#endif
        nrf_gpio_cfg_sense_input(pin_number, NRF_GPIO_PIN_NOPULL, sense_config);
        nrf_gpio_cfg_sense_set(pin_number, sense_config);
        nrf_delay_ms(1);
    }
#endif

#if defined(USR_MODULE_FUNCTION_USE_NFC) && defined(USR_MODULE_FUNCTION_USE_NFC_POWERON)
    NRF_NFCT->TASKS_SENSE = 1;
#endif
}

bool cfg_board_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_RESET:
            break;
    }
    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(cfg_board_shutdown_handler, 0);
void cfg_board_goto_power_down(void)
{
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF);
}

void cfg_board_pwr_mgmt_init(void)
{
    ret_code_t ret_code;
    ret_code = nrf_pwr_mgmt_init();
    cPrintLog(CDBG_FCTRL_DBG, "nrf_pwr_mgmt_init:%d\n", ret_code);
}

void cfg_board_early_init(cfg_board_shutdown_peripherals_func shutdown_peripherals_func)
{
    m_cfg_shutdown_peripherals_func = shutdown_peripherals_func;
    cfg_board_check_reset_reason();
#ifdef FEATURE_CFG_CHECK_BOOTSTRAP_PIN
    cfg_board_check_bootstrap_pin();
#else
    cfg_board_check_wifi_downloadmode();
#endif
#ifdef FEATURE_CFG_CHECK_NV_BOOT_MODE
    cfg_board_check_bootmode();
#endif
    cfg_board_gpio_set_default();
    cfg_board_check_bootloader();

    cfg_i2c_master_init();
    cfg_i2c_master_send_General_Call_Reset();
    
    cfg_bma250_early_init();
#ifndef CDEV_ACC_MODULE
    cfg_bma250_early_init();
    cfg_bma250_req_suppend_mode();
#endif
}

void cfg_board_init(void)
{
    cPrintLog(CDBG_FCTRL_DBG, "%s\n", __func__);
    cfg_board_resource_init();

    cfg_bma250_init();
#ifdef CDEV_ACC_MODULE
    if(cfg_peripheral_device_enable_status_get(PTYPE_ACC))
    {
        cfg_bma250_interrupt_init();
    }
    else
    {
        cfg_bma250_req_suppend_mode();
    }
#endif

    cfg_board_common_power_control(module_comm_pwr_pon_init, true);
    nrf_delay_ms(2);

#ifdef CDEV_WIFI_MODULE
    if(cfg_peripheral_device_enable_status_get(PTYPE_WIFI))
    {
        cWifi_prepare_start(m_module_peripheral_ID.wifi_MAC_STA);
        cfg_board_check_update_wifi_tx_pwr_tables();
    }
#endif

#if defined(CDEV_SIGFOX_MODULE) || defined(CDEV_SIGFOX_MONARCH_MODULE)
    cfg_sigfox_prepare_start();
    if(m_module_parameter.sigfox_recv_en)cfg_sigfox_downlink_on_off(true);
    if(m_module_parameter.sigfox_snek_testmode_enable)cfg_sigfox_set_senk_testmode_enable(true);
#endif

#ifdef CDEV_GPS_MODULE
    if(cfg_peripheral_device_enable_status_get(PTYPE_GPS))
    {
        cGps_resource_init();
        cGps_prepare_start();
#ifdef FEATURE_HDOP_VALUE_CHECK
        set_hdop_value_check(CGPS_BELOW_HDOP_9);
#endif
    }

#endif /* CDEV_GPS_MODULE */
    cfg_board_common_power_control(module_comm_pwr_pon_init, false);
#ifdef CDEV_RTC2_DATE_TIME_CLOCK
    RTC2_date_time_clock_init_N_start();
#endif
}

