#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_bas.h"
#include "nrf_sdm.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "peer_manager.h"
#include "nrf_drv_gpiote.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "hardfault.h"
#include "ble_dfu.h"
#include "ble_nus.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_mbr.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "main_peripherals.h"
#include "bma2x2.h"

extern const nrf_drv_twi_t m_twi;
extern volatile bool spi_xfer_done;

#if 1  //internal ACC
typedef enum
{
    InternalACC_NONE,
    InternalACC_BMA250E,
    InternalACC_BMA253,
    InternalACC_TypeMax
}InternalACC_Type_e;

int internalACC_Type = InternalACC_NONE;

static int internal_Acc_RegRead(uint8_t regAddr, uint8_t *regVal, uint8_t cnt);
static int internal_Acc_RegWrite(uint8_t regAddr,uint8_t * regVal, uint8_t cnt);

int internal_Acc_CheckInit(void)
{
    uint8_t i2c_buf[8];
    int ret = -1;
    uint8_t val_sleep_dur;
    const char *accName;

    if(internalACC_Type == InternalACC_NONE)
    {
        ret = internal_Acc_RegRead(BMA2x2_CHIP_ID_ADDR, i2c_buf, 1);
        if(ret == 0)
        {
            cPrintLog(CDBG_GSEN_DBG, "I_ACC ID:0x%02x\n", i2c_buf[0]);
            if((i2c_buf[0] == 0xf9) || (i2c_buf[0] == 0xfa))
            {
                if(i2c_buf[0] == 0xf9)
                {
                    internalACC_Type = InternalACC_BMA250E;
                    accName = "BMA250E";
                }
                else if(i2c_buf[0] == 0xfa)
                {
                    internalACC_Type = InternalACC_BMA253;
                    accName = "BMA253";
                }
                cPrintLog(CDBG_GSEN_INFO, "I_ACC is %s, Sensitivity:%d\n", accName, internal_Acc_Get_Sensitivity());

                //mode (LPM1)
                val_sleep_dur = BMA2x2_SLEEP_DURN_50MS;
                
                i2c_buf[0] = 0xB6;
                internal_Acc_RegWrite(BMA2x2_RST_ADDR, i2c_buf, 1);
                nrf_delay_ms(10);
                i2c_buf[0] = (0x10 | val_sleep_dur);
                internal_Acc_RegWrite(BMA2x2_MODE_CTRL_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);
                i2c_buf[0] = 0x00;
                internal_Acc_RegWrite(BMA2x2_LOW_NOISE_CTRL_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);
                i2c_buf[0] = (0x90 | val_sleep_dur);
                internal_Acc_RegWrite(BMA2x2_MODE_CTRL_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);
                i2c_buf[0] = 0xFF;
                internal_Acc_RegWrite(BMA2x2_FIFO_MODE_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);
                i2c_buf[0] = (0x50 | val_sleep_dur);
                internal_Acc_RegWrite(BMA2x2_MODE_CTRL_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);
                i2c_buf[0] = 0x03;
                internal_Acc_RegWrite(BMA2x2_RANGE_SELECT_ADDR, i2c_buf, 1);
                nrf_delay_ms(1);

                nrf_gpio_cfg_input(PIN_DEF_INT1_ACC, NRF_GPIO_PIN_PULLUP);  //open drain, internal pullup
                nrf_delay_ms(1);
                internal_Acc_Control_Open_Drain(true);
                nrf_delay_ms(1);
                ret = 0;
            }
            else
            {
                cPrintLog(CDBG_GSEN_ERR, "I_ACC Known ID!\n");
            }
        }
        else
        {
            cPrintLog(CDBG_GSEN_ERR, "I_ACC Init Err!\n");
        }
    }
    else
    {
        ret = 0;
    }
    return ret;
}

int internal_Acc_Shutdown(void)
{
    int ret = -1;
    uint8_t i2c_buf[8];

    internal_Acc_CheckInit();
    if(internalACC_Type == InternalACC_BMA250E || internalACC_Type == InternalACC_BMA253)
    {
        i2c_buf[0] = 0x20;  //enter deep_suspend
        ret = internal_Acc_RegWrite(BMA2x2_MODE_CTRL_ADDR, i2c_buf, 1);
        if(ret == 0)
        {
            cPrintLog(CDBG_GSEN_INFO, "I_ACC DEEP_SUSPEND!\n");
        }
    }
    else
    {
        cPrintLog(CDBG_GSEN_INFO, "I_ACC Shutdown Err:%d\n", internalACC_Type);
    }
    return ret;
}

bool internal_Acc_IsDetected(void)
{
    if(internalACC_Type == InternalACC_BMA250E || internalACC_Type == InternalACC_BMA253)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint16_t internal_Acc_Get_Sensitivity(void)
{
    return 256;
}

bool internal_Acc_Control_Open_Drain(bool bIsOpenDrain)
{
    uint8_t i2c_buf[8];
    static bool last_val = false;
    bool ret;
    if(bIsOpenDrain)
    {
        //INT out to open drain active low
        i2c_buf[0] = 0x02;  //int1_od to open drain, int1_lvl to active low
        internal_Acc_RegWrite(BMA2x2_INTR_SET_ADDR, i2c_buf, 1);  //INT_OUT_CTRL in datasheet
    }
    else
    {
        //INT out to push-pull active Low
        i2c_buf[0] = 0x01;
        internal_Acc_RegWrite(BMA2x2_INTR_SET_ADDR, i2c_buf, 1);  //INT_OUT_CTRL in datasheet
    }

    ret = last_val;
    last_val = bIsOpenDrain;
    return last_val;
}

int internal_Acc_Get_XYZ(int16_t *x, int16_t *y, int16_t *z)
{
    int ret = -1;
    uint8_t i2c_buf[8];
    int16_t accel_x, accel_y, accel_z;

    if(internalACC_Type == InternalACC_BMA250E || internalACC_Type == InternalACC_BMA253)
    {
        if(internal_Acc_RegRead(BMA2x2_X_AXIS_LSB_ADDR, i2c_buf, 6) == 0)
        {
            if(internalACC_Type == InternalACC_BMA250E)
            {
                accel_x = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[1]))
                                << ((uint8_t)8)) |
                                (i2c_buf[0] &
                                (0xC0)));
                accel_x = (accel_x >> ((uint8_t)6));

                /* read the y i2c_buf*/
                accel_y = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[3]))
                                << (uint8_t)8) |
                                (i2c_buf[2] &
                                (0xC0)));
                accel_y = (accel_y >> (uint8_t)6);

                /* read the z i2c_buf*/
                accel_z = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[5]))
                                << (uint8_t)8) |
                                (i2c_buf[4] &
                                (0xC0)));
                accel_z = (accel_z >> (uint8_t)6);
            }
            else if(internalACC_Type == InternalACC_BMA253)
            {
                accel_x = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[1]))
                                << (uint8_t)8) |
                                (i2c_buf[0] &
                                (0xF0)));
                accel_x = (accel_x >> ((uint8_t)6));

                /* read the y i2c_buf*/
                accel_y = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[3]))
                                << (uint8_t)8) |
                                (i2c_buf[2] &
                                (0xF0)));
                accel_y = (accel_y >> (uint8_t)6);

                /* read the z i2c_buf*/
                accel_z = (int16_t)((((int32_t)((int8_t)
                                i2c_buf[5]))
                                << (uint8_t)8) |
                                (i2c_buf[4] &
                                (0xF0)));
                accel_z = (accel_z >> (uint8_t)6);
            }
    
            if(x)*x = accel_x;
            if(y)*y = accel_y;
            if(z)*z = accel_z;
        }
        ret = 0;
    }
    return ret;
}

static int internal_Acc_RegRead(uint8_t regAddr, uint8_t *regVal, uint8_t cnt)
{
    uint8_t i2c_buf[8];
    uint32_t timeout;
    int ret = -1;
    int twiApiRet;

    i2c_buf[0] = regAddr;
    timeout = CFG_I2C_TIMEOUT;
    spi_xfer_done = 0;
    twiApiRet = nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1, (uint8_t const *)i2c_buf, 1, true);
    if(twiApiRet)cPrintLog(CDBG_GSEN_ERR, "I_ACC rd Twi Err 2!\n");
    while((!spi_xfer_done) && --timeout);
    if(timeout)
    {
        timeout = CFG_I2C_TIMEOUT;
        spi_xfer_done = 0;
        twiApiRet = nrf_drv_twi_rx(&m_twi,BMA2x2_I2C_ADDR1, i2c_buf, cnt);
        if(twiApiRet)cPrintLog(CDBG_GSEN_ERR, "I_ACC rd Twi Err 3!\n");
        while((!spi_xfer_done) && --timeout);
        if(timeout)
        {
            memcpy(regVal, i2c_buf, cnt);
            ret = 0;
        }
        else
        {
            cPrintLog(CDBG_GSEN_ERR, "I_ACC readData Timeout\n");
        }
    }
    else
    {
        cPrintLog(CDBG_GSEN_ERR, "I_ACC SetRegAddr Timeout\n");
    }
    return ret;
}

static int internal_Acc_RegWrite(uint8_t regAddr,uint8_t * regVal, uint8_t cnt)
{
    uint8_t i2c_buf[8];
    uint32_t timeout;
    int ret = -1;
    int twiApiRet;

    i2c_buf[0] = regAddr;
    memcpy(&i2c_buf[1], regVal, cnt);
    timeout = CFG_I2C_TIMEOUT;
    spi_xfer_done = 0;
    twiApiRet = nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1, (uint8_t const *)i2c_buf, cnt +1, false);
    if(twiApiRet)cPrintLog(CDBG_GSEN_ERR, "I_ACC wr Twi Err 2!\n");
    while((!spi_xfer_done) && --timeout);
    if(timeout)
    {
        memcpy(regVal, i2c_buf, cnt);
        ret = 0;
    }
    else
    {
        cPrintLog(CDBG_GSEN_ERR, "I_ACC writeData Timeout\n");
    }

    return ret;
}
#endif  //internal ACC

