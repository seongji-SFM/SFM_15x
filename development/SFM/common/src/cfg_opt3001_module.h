/* Copyright 2015 The Chromium OS Authors. All rights reserved. 
  * Use of this source code is governed by a BSD-style license that can be 
  * found in the LICENSE file. 
  * 
  * TI OPT3001 light sensor driver 
  */ 

#ifndef __CROS_EC_ALS_OPT3001_H 
#define __CROS_EC_ALS_OPT3001_H 

/* Ambient Light Sensor address */
#define OPT3001_I2C_ADDR OPT3001_I2C_ADDR1

/*   CONFIG REGISTER BITS: RN3 RN2 RN1 RN0 CT M1 M0 OVF CRF FH FL L Pol ME FC1 FC0 
RN3 to RN0 = Range select: 
                  1100 by default, enables auto-range  
CT = Conversion time bit 
                  0 = 100ms conversion time 
                  1 = 800ms conversion time (default) 
M1 to M0 = Mode bits 
                  00 = Shutdown mode 
                  01 = Single shot mode 
                  10 = Continuous conversion (default) 
                  11 = Continuous conversion 
OVF (Bit 8)     Overflow flag. When set the conversion result is overflown. 
CRF (Bit 7)     Conversion ready flag. Sets at end of conversion. Clears by read or write of the Configuration register. 
FH (Bit 6)      Flag high bit. Read only. Sets when result is higher that TH register. Clears when Config register is  
               read or when Latch bit is 0 and the result goes bellow TH register. 
FL (Bit 5)      Flag low bit. Read only. Sets when result is lower that TL register. Clears when Config register is read  
                  or when Latch bit is 0 and the result goes above TL register. 
L (Bit 4)       Latch bit. Read/write bit. Default 1, Controls Latch/transparent functionality of FH and FL bits. When  
                  L = 1 the Alert pin works in window comparator mode with Latched functionality When L = 0 the Alert pin  
                 works in transparent mode and the two limit registers provide the hysteresis. 
Pol (Bit 3)     Polarity. Read/write bit. Default 0, Controls the active state of the Alert pin. Pol = 0 means Alert  
                  active low. 
ME (Bit 2)      Exponent mask. In fixed range modes masks the exponent bits in the result register to 0000. 
FC1 to FC0  -   Fault count bits. Read/write bits. Default 00 - the first fault will trigger the alert pin. 
*/ 
  
#define OPT3001_CFG_FL          (1 << 5) 
#define OPT3001_CFG_FH          (1 << 6) 
#define OPT3001_CFG_CRF         (1 << 7) 
#define OPT3001_CFG_OVF         (1 << 8) 
#define OPT3001_CFG_L           (1 << 4) 
#define OPT3001_CFG_POL         (1 << 3) 
#define OPT3001_CFG_ME          (1 << 2) 
#define OPT3001_CFG_CT          (1 << 11) 
#define OPT3001_CFG_FC_SHIFT    0 
#define OPT3001_CFG_FC_MASK     (0x03 << OPT3001_CFG_FC_SHIFT) 
#define OPT3001_CFG_M_SHIFT     9 
#define OPT3001_CFG_M_MASK      (0x03 << OPT3001_CFG_M_SHIFT) 
#define OPT3001_CFG_RN_SHIFT    12 
#define OPT3001_CFG_RN_MASK     (0xf << OPT3001_CFG_RN_SHIFT) 


#define DEFAULT_CONFIG_800    0b1100110000010000 // 800ms // CC10
#define DEFAULT_CONFIG_800_OS 0b1100101000010000 // 800ms, one shot  // CA10


#define DEFAULT_CONFIG_100    0b1100010000010000 // 100ms  // C410
#define DEFAULT_CONFIG_100_OS 0b1100001000010000 // 100ms, one shot  // C210


#define DEFAULT_CONFIG_SHDWN  0xC810 // Shut down converter // 1100100000010000


/*---------------------------------------------------------------------------*/
/* Register values */
#define MANUFACTURER_ID     0x5449 /* TI */
#define DEVICE_ID           0x3001 /* Opt 3001 */
#define CONFIG_RESET        0xC810
#define CONFIG_TEST         0xCC10
#define CONFIG_ENABLE       0x10CC /* 0xCC10 */
#define CONFIG_DISABLE      0x108C /* 0xC810 */

#define DEFAULT_CONFIG_SET         0xCC10 // Conversion time field 800ms
#define DEFAULT_CONFIG_SET_100MS   0xC410 // Conversion time field 100ms

/*---------------------------------------------------------------------------*/
/* Bit values */
#define DATA_RDY_BIT 0x0080 /* Data ready */
/*---------------------------------------------------------------------------*/
/* Register length */
#define REGISTER_LENGTH 2
/*---------------------------------------------------------------------------*/
/* Sensor data size */
#define DATA_LENGTH 2
/*---------------------------------------------------------------------------*/
#define SENSOR_STATUS_DISABLED 0
#define SENSOR_STATUS_NOT_READY 1
#define SENSOR_STATUS_ENABLED 2



/* I2C interface */ 
#define OPT3001_I2C_ADDR1       0x44 
#define OPT3001_I2C_ADDR2       0x45 
#define OPT3001_I2C_ADDR3       0x46 
#define OPT3001_I2C_ADDR4       0x47 

/* OPT3001 registers */ 
#define OPT3001_REG_RESULT      0x00 
#define OPT3001_REG_CONFIGURE   0x01 
#define OPT3001_RANGE_OFFSET    12 
#define OPT3001_RANGE_MASK      0x0fff 
#define OPT3001_MODE_OFFSET     9 
#define OPT3001_MODE_MASK       0xf9ff 
enum opt3001_mode { 
OPT3001_MODE_SUSPEND, 
    OPT3001_MODE_FORCED, 
    OPT3001_MODE_CONTINUOUS, 
}; 

#define OPT3001_REG_INT_LIMIT_LSB   0x02 // Low
#define OPT3001_REG_INT_LIMIT_MSB   0x03 // High 
#define OPT3001_REG_MAN_ID          0x7e 
#define OPT3001_REG_DEV_ID          0x7f 

#define OPT3001_LOW_DATA_DEFAULT  0x0000
#define OPT3001_HIGH_DATA_DEFAULT 0xBFFF

#define OPT3001_INTR_DATA           OPT3001_RANGE_655P20_LUX // int low & High
#define OPT3001_INTR_LOW_DEFAULT    OPT3001_LOW_DATA_DEFAULT 
#define OPT3001_INTR_HIGH_DEFAULT   OPT3001_HIGH_DATA_DEFAULT 

/* OPT3001 register values */ 
#define OPT3001_MANUFACTURER_ID     0x5449 
#define OPT3001_DEVICE_ID           0x3001 

/* Min and Max sampling frequency in mHz */ 
#define OPT3001_LIGHT_MIN_FREQ          1250 
#define OPT3001_LIGHT_MAX_FREQ          10000 

/* OPT3001 Full-Scale Range */ 
enum opt3001_range { 
    OPT3001_RANGE_40P95_LUX = 0, 
    OPT3001_RANGE_81P90_LUX, 
    OPT3001_RANGE_163P80_LUX, 
    OPT3001_RANGE_327P60_LUX, 
    OPT3001_RANGE_655P20_LUX, 
    OPT3001_RANGE_1310P40_LUX, 
    OPT3001_RANGE_2620P80_LUX, 
    OPT3001_RANGE_5241P60_LUX, 
    OPT3001_RANGE_10483P20_LUX, 
    OPT3001_RANGE_20966P40_LUX, 
    OPT3001_RANGE_41932P80_LUX, 
    OPT3001_RANGE_83865P60_LUX,
    OPT3001_RANGE_AUTOMATIC_FULL_SCALE, 
}; 

/**@brief Structure for opt3001 state machine. */
typedef enum
{
    NONE_ALS,
    ALS_SET_S,
    ALS_SET_R,
    ALS_READ_DATA_S,
    ALS_READ_DATA_R,
    ALS_READ_ONLY_S,
    ALS_READ_ONLY_R,
    ALS_SLEEP_S, 
    ALS_SLEEP_R,
    ALS_INTERRUPT_S,
    ALS_INTERRUPT_R,
    ALS_INTERRUPT_CLEAR_S,
    ALS_INTERRUPT_CLEAR_R,
    EXIT_ALS
} opt3001_state_s;

typedef enum
{
    ALS_DISABLE,
    ALS_ENABLE,
    ALS_NONE,
} opt3001_sensor_s;

typedef enum
{
    ALS_INTR_NONE,
    ALS_INTR_LOW,
    ALS_INTR_HIGH,
} opt3001_event_intr_s;

typedef enum
{
    ALS_INTR_SET_DISABLE,        /* Interrupt low & high disable */
    ALS_INTR_SET_LOW,
    ALS_INTR_SET_HIGH,
} opt3001_set_intr_s;

/* List of common error codes that can be returned */
enum ec_error_list {
    /* Success - no error */
    EC_SUCCESS = 0,
    /* Unknown error */
    EC_ERROR_UNKNOWN = 1,
    /* Function not implemented yet */
    EC_ERROR_UNIMPLEMENTED = 2,
    /* Overflow error; too much input provided. */
    EC_ERROR_OVERFLOW = 3,
    /* Timeout */
    EC_ERROR_TIMEOUT = 4,
    /* Invalid argument */
    EC_ERROR_INVAL = 5,
    /* Already in use, or not ready yet */
    EC_ERROR_BUSY = 6,
    /* Access denied */
    EC_ERROR_ACCESS_DENIED = 7,
    /* Failed because component does not have power */
    EC_ERROR_NOT_POWERED = 8,
    /* Failed because component is not calibrated */
    EC_ERROR_NOT_CALIBRATED = 9,
    /* Failed because CRC error */
    EC_ERROR_CRC = 10,
    /* Invalid console command param (PARAMn means parameter n is bad) */
    EC_ERROR_PARAM1 = 11,
    EC_ERROR_PARAM2 = 12,
    EC_ERROR_PARAM3 = 13,
    EC_ERROR_PARAM4 = 14,
    EC_ERROR_PARAM5 = 15,
    EC_ERROR_PARAM6 = 16,
    EC_ERROR_PARAM7 = 17,
    EC_ERROR_PARAM8 = 18,
    EC_ERROR_PARAM9 = 19,
    /* Wrong number of params */
    EC_ERROR_PARAM_COUNT = 20,
    /* Interrupt event not handled */
    EC_ERROR_NOT_HANDLED = 21,
    /* Data has not changed */
    EC_ERROR_UNCHANGED = 22,
    /* Memory allocation */
    EC_ERROR_MEMORY_ALLOCATION = 23,

    /* Verified boot errors */
    EC_ERROR_VBOOT_SIGNATURE = 0x1000, /* 4096 */
    EC_ERROR_VBOOT_SIG_MAGIC = 0x1001,
    EC_ERROR_VBOOT_SIG_SIZE = 0x1002,
    EC_ERROR_VBOOT_SIG_ALGORITHM = 0x1003,
    EC_ERROR_VBOOT_HASH_ALGORITHM = 0x1004,
    EC_ERROR_VBOOT_SIG_OFFSET = 0x1005,
    EC_ERROR_VBOOT_DATA_SIZE = 0x1006,

    /* Verified boot key errors */
    EC_ERROR_VBOOT_KEY = 0x1100,
    EC_ERROR_VBOOT_KEY_MAGIC = 0x1101,
    EC_ERROR_VBOOT_KEY_SIZE = 0x1102,

    /* Verified boot data errors */
    EC_ERROR_VBOOT_DATA = 0x1200,
    EC_ERROR_VBOOT_DATA_VERIFY = 0x1201,

    /* Module-internal error codes may use this range.   */
    EC_ERROR_INTERNAL_FIRST = 0x10000,
    EC_ERROR_INTERNAL_LAST =  0x1FFFF
};

/* Host command response codes (16-bit).  Note that response codes should be
* stored in a uint16_t rather than directly in a value of this type.
*/
enum ec_status {
    EC_RES_SUCCESS = 0,
    EC_RES_INVALID_COMMAND = 1,
    EC_RES_ERROR = 2,
    EC_RES_INVALID_PARAM = 3,
    EC_RES_ACCESS_DENIED = 4,
    EC_RES_INVALID_RESPONSE = 5,
    EC_RES_INVALID_VERSION = 6,
    EC_RES_INVALID_CHECKSUM = 7,
    EC_RES_IN_PROGRESS = 8,     /* Accepted, command in progress */
    EC_RES_UNAVAILABLE = 9,     /* No response available */
    EC_RES_TIMEOUT = 10,        /* We got a timeout */
    EC_RES_OVERFLOW = 11,       /* Table / data overflow */
    EC_RES_INVALID_HEADER = 12,     /* Header contains invalid data */
    EC_RES_REQUEST_TRUNCATED = 13,  /* Didn't get the entire request */
    EC_RES_RESPONSE_TOO_BIG = 14,   /* Response was too big to handle */
    EC_RES_BUS_ERROR = 15,      /* Communications bus error */
    EC_RES_BUSY = 16        /* Up but too busy.  Should retry */
};

#define OPT3001_GET_DATA(_s)    ((struct opt3001_drv_data_t *)(_s)->drv_data) 

struct opt3001_drv_data_t { 
    enum opt3001_range range; 
    int rate; 
    int last_value; 
    int attenuation; 
}; 


typedef void (*opt3001_attach_callback)(void);

/**
 * @brief       Function for opt3001 timer create
 */
void cfg_opt3001_timer_create(void);

/**
 * @brief       Function for opt3001 timer start
 */
void cfg_opt3001_timers_start(void);

/**
 * @brief       Function for opt3001 timer stop
 */
void cfg_opt3001_timers_stop(void);

/**
 * @brief       Function for opt3001 get state
 */
opt3001_state_s opt3001_get_state(void);

/**
 * @brief       Function for opt3001 set state
 */
void opt3001_set_state(opt3001_state_s m_state);
void opt3001_i2c_init(void);
void cfg_opt3001_als_init(void);
int opt3001_get_lux(uint32_t *get_lux);
int opt3001_get_reg_result(void);
int opt3001_get_configuration(void);
int opt3001_set_shutdown(void);
int opt3001_set_configuration(uint16_t config);
int opt3001_set_clear_intr(void);
void opt3001_set_clear_state(void);
bool opt3001_get_enable_intr(void);
bool opt3001_intr_wakeup_reason(void);

#ifdef CONFIG_CMD_I2C_STRESS_TEST_ALS 
extern struct i2c_stress_test_dev opt3001_i2c_stress_test_dev; 
#endif 

#endif  /* __CROS_EC_ALS_OPT3001_H */ 

