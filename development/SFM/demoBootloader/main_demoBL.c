/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"

#if defined(FEATURE_WISOL_DEVICE) && defined(FEATURE_WISOL_BOOTLOADER)
#include "cfg_config_defines.h"
#define CDBG_LOG_INSTANCE
#include "cfg_dbg_log.h"

#include "nrf_gpio.h"
#include "nrf_drv_uart.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"

#ifdef FEATURE_DFU_BOTH_BLE_AND_SEIRAL  //support DFU ble and serial (support "development/SFM/tools/serial_downloader")
#define bootloader_secure_Developer_logs (0)
//#define TEST_FEAUTRE_DFU_SERIAL_TARGET_NRF52_DK  //test for nRF52 DK board

const char m_cfg_bl_ver[] = {CDEV_BL_VER};
const char m_cfg_bl_ver_size = sizeof(m_cfg_bl_ver);
const char m_cfg_bl_build_date[] = __DATE__;
const char m_cfg_bl_build_time[] = __TIME__;

extern bool nrf_serial_dfu_enable;
extern bool nrf_ble_dfu_enable;
extern const uint8_t pk[64];  //dfu_public_key.c

#ifdef TEST_FEAUTRE_DFU_SERIAL_TARGET_NRF52_DK
#define CHECK_UART_RX_PIN_DEF RX_PIN_NUMBER
#define CHECK_UART_RX_PIN_INPUT_PULL_CFG NRF_GPIO_PIN_NOPULL
#else
#define CHECK_UART_RX_PIN_DEF PIN_DEF_DTM_RX
#define CHECK_UART_RX_PIN_INPUT_PULL_CFG NRF_GPIO_PIN_PULLDOWN
#endif


#define UART_CMD_WAIT_TICK_CNT 5  //tick delay 100ms
#define UART_RX_TX_BUF_SIZE 128

#define check_UART_CMD_ENTER_PARSER             ('W')

//OP CODE
#define check_UART_CMD_BOOT                     ('X')
#define check_UART_CMD_REBOOT                   ('R')
#define check_UART_CMD_ENTER_SERIAL_DFU         ('S')
#define check_UART_CMD_ENTER_BLE_DFU            ('B')
#define check_UART_CMD_TEST_LOG                 ('L')
#define check_UART_CMD_GET_BL_VER               ('V')
#define check_UART_CMD_CONTROL_DBG_IF           ('D')  //TODO  //need hash

//RESP
#define check_UART_RESP_OK                      ('K')
#define check_UART_RESP_ERROR                   ('E')

typedef enum
{
    check_UART_cmd_processor_boot_r,
    check_UART_cmd_processor_enter_serial_DFU_r,
    check_UART_cmd_processor_enter_ble_DFU_r,
    check_UART_cmd_processor_reboot_request_r,
    check_UART_cmd_processor_result_emax
}check_UART_cmd_processor_result_e;

typedef enum
{
    check_UART_wait_start_s,
    check_UART_wait_cmd_s,
    check_UART_wait_dbg_if_ctrl_s,
    check_UART_cmd_proc_enter_serial_DFU_s,
    check_UART_cmd_proc_enter_ble_DFU_s,
    check_UART_cmd_proc_reboot_request_s,
    check_UART_parser_exit_s,
    check_UART_parser_state_e_max
}check_UART_parser_state_e;

static nrf_drv_uart_t check_UART_instance =  NRF_DRV_UART_INSTANCE(0);
static uint8_t check_UART_RX_buf[4];  //just use 1 byte
static uint8_t check_UART_TX_buf[UART_RX_TX_BUF_SIZE];
static volatile int check_UART_state = check_UART_wait_start_s;
static uint8_t check_UART_param_buf[64];
static uint32_t check_UART_param_idx;
static uint32_t check_UART_param_size;

void hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin)
{
    int i;
    uint8_t h_val, l_val;
    if((HexadeccoimalByteCnt < 0) || ((HexadeccoimalByteCnt % 2) != 0))
        return;

    for(i = 0; i < HexadeccoimalByteCnt; i+=2)
    {
        h_val = 0;
        l_val = 0;
        if(pHexadecimal[i] >= 'A' && pHexadecimal[i] <= 'F')
            h_val = pHexadecimal[i]-'A'+10;
        else if(pHexadecimal[i] >= 'a' && pHexadecimal[i] <= 'f')
            h_val = pHexadecimal[i]-'a'+10;
        else if(pHexadecimal[i] >= '0' && pHexadecimal[i] <= '9')
            h_val = pHexadecimal[i]-'0';
        
        if(pHexadecimal[i+1] >= 'A' && pHexadecimal[i+1] <= 'F')
            l_val = pHexadecimal[i+1]-'A'+10;
        else if(pHexadecimal[i+1] >= 'a' && pHexadecimal[i+1] <= 'f')
            l_val = pHexadecimal[i+1]-'a'+10;
        else if(pHexadecimal[i+1] >= '0' && pHexadecimal[i+1] <= '9')
            l_val = pHexadecimal[i+1]-'0';

        *pBin++ = (h_val << 4 | l_val);
    }
}

typedef enum
{
    cfg_dfu_led_ctrl_off,
    cfg_dfu_led_ctrl_serial_ctrl_connected,
    cfg_dfu_led_ctrl_wait_DFU_download,
    cfg_dfu_led_ctrl_updating,
    cfg_dfu_led_ctrl_error,
    cfg_dfu_led_ctrl_type_max
}cfg_dfu_led_ctrl_type_e;
#ifdef USR_MODULE_GPIO_DEF_BLE_LED
#define CFG_DFU_INIT() {nrf_gpio_cfg_output(USR_MODULE_GPIO_DEF_BLE_LED); nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, 0); }
#define CFG_DFU_LED_ON() nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON)
#define CFG_DFU_LED_OFF() nrf_gpio_pin_write(USR_MODULE_GPIO_DEF_BLE_LED, !USR_MODULE_GPIO_DEF_BLE_LED_HIGH_TO_ON)
#define CFG_DFU_UNINIT() nrf_gpio_cfg_default(USR_MODULE_GPIO_DEF_BLE_LED)
#else
#define CFG_DFU_INIT()
#define CFG_DFU_LED_ON()
#define CFG_DFU_LED_OFF()
#define CFG_DFU_UNINIT()
#endif
static bool resource_init = false;
static uint32_t bl_tick_counter = 0;
static cfg_dfu_led_ctrl_type_e led_ctrl_type;

APP_TIMER_DEF(m_bl_tick_timer_id);
static void timeout_handler_bl_tick_timer(void * p_context)
{
    bl_tick_counter++;

    switch(led_ctrl_type)
    {
        case cfg_dfu_led_ctrl_serial_ctrl_connected:  //Long  On / Short Off
            if((bl_tick_counter % 20) == 0)
                CFG_DFU_LED_OFF();
            else if((bl_tick_counter % 5) == 0)
                CFG_DFU_LED_ON();
            break;
        case cfg_dfu_led_ctrl_wait_DFU_download:   //Long  On / Long  Off
            if((bl_tick_counter % 40) == 0)
                CFG_DFU_LED_ON();
            else if((bl_tick_counter % 20) == 0)
                CFG_DFU_LED_OFF();
            break;

        case cfg_dfu_led_ctrl_updating:  //Short On / Short Off
            if((bl_tick_counter % 4) == 0)
                CFG_DFU_LED_ON();
            else if((bl_tick_counter % 2) == 0)
                CFG_DFU_LED_OFF();
            break;
        case cfg_dfu_led_ctrl_error:  //Short On / Long  Off
            if((bl_tick_counter % 20) == 0)
                CFG_DFU_LED_ON();
            else if((bl_tick_counter % 5) == 0)
                CFG_DFU_LED_OFF();
            break;

        default:
            break;
    }
}
static void cfg_dfu_led_ctrl(cfg_dfu_led_ctrl_type_e type)
{
    led_ctrl_type = type;
    if(type == cfg_dfu_led_ctrl_off)
    {
        CFG_DFU_LED_OFF();
    }
}
static void cfg_dfu_resource_check_init(void)
{
    if(!resource_init)
    {
        resource_init = true;
#if (bootloader_secure_Developer_logs)
        cPrintLog(CDBG_MAIN_LOG, "BL ADDR [mbr:%p, uicr:%p]\n", (*(uint32_t *)MBR_BOOTLOADER_ADDR), *MBR_UICR_BOOTLOADER_ADDR);
#endif
        CFG_DFU_INIT();
        app_timer_init();
        app_timer_create(&m_bl_tick_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler_bl_tick_timer);
        app_timer_start(m_bl_tick_timer_id, APP_TIMER_TICKS(100), NULL);
    }
}

static void cfg_dfu_resource_check_uninit(void)
{
    if(resource_init)
    {
        resource_init = false;
        app_timer_stop(m_bl_tick_timer_id);
    }
}

static bool check_UART_connect_with_RX_pin(void)
{
    int check_cnt = 10, number_of_times_OK = 7;
    int high_cnt = 0;
    int loop = 0;
    nrf_gpio_cfg_input(CHECK_UART_RX_PIN_DEF, CHECK_UART_RX_PIN_INPUT_PULL_CFG);
    loop = check_cnt;
    do
    {
        nrf_delay_ms(1);
        if(nrf_gpio_pin_read(CHECK_UART_RX_PIN_DEF))
        {
            high_cnt++;
        }
    }while(--loop);
    nrf_gpio_cfg_default(CHECK_UART_RX_PIN_DEF);
    if(high_cnt >= number_of_times_OK)
    {
#if (bootloader_secure_Developer_logs)
        cPrintLog(CDBG_MAIN_LOG, "Uart Det[%d/%d]\n", high_cnt, check_cnt);
#endif
        return true;
    }
    return false;
}

static void check_UART_send_OK_resp(void)

{
    check_UART_TX_buf[0] = check_UART_RESP_OK;
    (void)nrf_drv_uart_tx(&check_UART_instance, check_UART_TX_buf, 1);
}

static void check_UART_send_ERROR_resp(void)

{
    check_UART_TX_buf[0] = check_UART_RESP_ERROR;
    (void)nrf_drv_uart_tx(&check_UART_instance, check_UART_TX_buf, 1);
}

static void check_UART_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_TX_DONE:
            break;

        case NRF_DRV_UART_EVT_RX_DONE:
            if(p_event->data.rxtx.bytes == 1)  //just support 1byte read
            {
                switch(check_UART_state)
                {
                    case check_UART_wait_start_s:
                        if((p_event->data.rxtx.p_data[0] == check_UART_CMD_ENTER_PARSER))
                        {
#if (bootloader_secure_Developer_logs)
                            cPrintLog(CDBG_MAIN_LOG, "Uart CMD Mode\n");
#endif
                            check_UART_state = check_UART_wait_cmd_s;
                            check_UART_send_OK_resp();
                        }
                        break;
                    case check_UART_wait_cmd_s:
#if (bootloader_secure_Developer_logs)
                        cPrintLog(CDBG_MAIN_LOG, "Uart CMD 0x%02x\n", p_event->data.rxtx.p_data[0]);
#endif
                        switch(p_event->data.rxtx.p_data[0])
                        {
                            case check_UART_CMD_ENTER_PARSER:
                                check_UART_send_OK_resp();
                                break;
                            case check_UART_CMD_BOOT:
                                check_UART_state = check_UART_parser_exit_s;
                                check_UART_send_OK_resp();
                                break;
                            case check_UART_CMD_ENTER_SERIAL_DFU:
                                check_UART_state = check_UART_cmd_proc_enter_serial_DFU_s;
                                check_UART_send_OK_resp();
                                break;
                            case check_UART_CMD_ENTER_BLE_DFU:
                                check_UART_state = check_UART_cmd_proc_enter_ble_DFU_s;
                                check_UART_send_OK_resp();
                                break;
                            case check_UART_CMD_REBOOT:
                                check_UART_state = check_UART_cmd_proc_reboot_request_s;
                                check_UART_send_OK_resp();
                                break;
                            case check_UART_CMD_TEST_LOG:
                                cPrintLog(CDBG_MAIN_LOG, "DFU Test Log! Build Date:%s %s\n",  (const char *)m_cfg_bl_build_date, (const char *)m_cfg_bl_build_time);
                                break;
                            case check_UART_CMD_GET_BL_VER:
                                {
                                    int ver_str_len;
                                    ver_str_len = snprintf((char *)check_UART_TX_buf, sizeof(check_UART_TX_buf), "%s,%s %s\r\n", m_cfg_bl_ver, m_cfg_bl_build_date, m_cfg_bl_build_time);
                                    (void)nrf_drv_uart_tx(&check_UART_instance, check_UART_TX_buf, ver_str_len);
                                }
                                break;
                            case check_UART_CMD_CONTROL_DBG_IF:
                                check_UART_param_idx=0;
                                check_UART_param_size=9;
                                check_UART_state = check_UART_wait_dbg_if_ctrl_s;
                                break;
                                
                            default:
                                check_UART_send_ERROR_resp();
                                break;
                        }
                        break;

                    case check_UART_wait_dbg_if_ctrl_s:
                        check_UART_param_buf[check_UART_param_idx++] = p_event->data.rxtx.p_data[0];
                        if(check_UART_param_idx>=check_UART_param_size)
                        {
                            bool result = false;
                            uint8_t key_buf[4];
                            hexadecimal_2_bin((const char *)&check_UART_param_buf[1], 8, key_buf);
                            if(memcmp((void *)key_buf, (void *)pk, 4) == 0)
                            {
                                uint32_t  APPROTECT_old_val;
                                uint32_t  APPROTECT_new_val;
                                APPROTECT_old_val = NRF_UICR->APPROTECT;
                                if(check_UART_param_buf[0]=='E')
                                {
                                    //If it does not work, you probably does not have a chip with pre-programmed factory code, as mentioned in the post.
                                    //So, we refer to nrf52xxx_uicr.flm.
                                    int i;
                                    uint32_t *uicrPtr;
                                    NRF_UICR_Type *p_uicr_memory_layout;
                                    APPROTECT_new_val = 0xffffffff;
                                    cPrintLog(CDBG_MAIN_LOG, "Enable DAP. 0x%08x to 0x%08x\n", APPROTECT_old_val, APPROTECT_new_val);

                                    if(APPROTECT_old_val != APPROTECT_new_val)
                                    {
                                        static uint32_t check_UART_UICR_buf[((0x20C+4)/*UICR size*/ + 16/*dummy*/)/4];  //not support nRF82832 
                                        static uint32_t check_UART_UICR_item_cnt = (sizeof(check_UART_UICR_buf)/sizeof(uint32_t));  // //not support nRF82832 

                                        memcpy(check_UART_UICR_buf, (void *)NRF_UICR_BASE, sizeof(check_UART_UICR_buf));

                                        //change APPROTECT for dap enable
                                        p_uicr_memory_layout = (NRF_UICR_Type *)check_UART_UICR_buf;
                                        p_uicr_memory_layout->APPROTECT = APPROTECT_new_val;
                                        //If it does not work, you probably does not have a chip with pre-programmed factory code, as mentioned in the post.
                                        //So, we refer to nrf52xxx_uicr.flm.
                                        { //erase uisr
                                            //ref nrf52xxx_uicr.flm (e.g. C:\Keil_v5\ARM\PACK\NordicSemiconductor\nRF_DeviceFamilyPack\8.15.0\Flash)
                                            //fromelf.exe nrf52xxx_uicr.flm --text -c -> disassemble code

                                            //int Init(unsigned long adr, unsigned long clk, unsigned long fnc)  //adr:  Device Base Address, clk:  Clock Frequency (Hz), fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
                                            //Init
                                            //0x00000008:    4830        0H      LDR      r0,[pc,#192] ; [0xcc] = 0x4001e504
                                            //0x0000000a:    2a01        .*      CMP      r2,#1
                                            //0x0000000c:    d005        ..      BEQ      0x1a ; Init + 18
                                            //0x0000000e:    2a02        .*      CMP      r2,#2
                                            //0x00000010:    d005        ..      BEQ      0x1e ; Init + 22
                                            //0x00000012:    2a03        .*      CMP      r2,#3
                                            //0x00000014:    d00b        ..      BEQ      0x2e ; Init + 38
                                            //0x00000016:    2001        .       MOVS     r0,#1
                                            //0x00000018:    4770        pG      BX       lr
                                            //0x0000001a:    2102        .!      MOVS     r1,#2
                                            //0x0000001c:    e000        ..      B        0x20 ; Init + 24
                                            //0x0000001e:    2101        .!      MOVS     r1,#1
                                            //0x00000020:    6001        .`      STR      r1,[r0,#0]
                                            //0x00000022:    482b        +H      LDR      r0,[pc,#172] ; [0xd0] = 0x4001e400
                                            //0x00000024:    6801        .h      LDR      r1,[r0,#0]
                                            //0x00000026:    2900        .)      CMP      r1,#0
                                            //0x00000028:    d0fc        ..      BEQ      0x24 ; Init + 28
                                            //0x0000002a:    2000        .       MOVS     r0,#0
                                            //0x0000002c:    4770        pG      BX       lr
                                            //0x0000002e:    2100        .!      MOVS     r1,#0
                                            //0x00000030:    e7f6        ..      B        0x20 ; Init + 24

                                            //LDR      r0,[pc,#192] ; [0xcc] = 0x4001e504
                                            //CMP      r2,#1
                                            //BEQ      0x1a ; Init + 18
                                            //MOVS     r1,#2
                                            //B        0x20 ; Init + 24
                                            //STR      r1,[r0,#0]
                                            *((uint32_t *)0x4001e504) = 2;

                                            //LDR      r0,[pc,#172] ; [0xd0] = 0x4001e400
                                            //LDR      r1,[r0,#0]
                                            //CMP      r1,#0
                                            //BEQ      0x24 ; Init + 28
                                            while(*((uint32_t *)0x4001e400) == 0);

                                            //MOVS     r0,#0
                                            //BX       lr
                                            //return 0

                                            //int EraseChip(void)
                                            //EraseChip
                                            //0x00000044:    4921        !I      LDR      r1,[pc,#132] ; [0xcc] = 0x4001e504
                                            //0x00000046:    2001        .       MOVS     r0,#1
                                            //0x00000048:    3110        .1      ADDS     r1,r1,#0x10
                                            //0x0000004a:    6008        .`      STR      r0,[r1,#0]
                                            //0x0000004c:    4820         H      LDR      r0,[pc,#128] ; [0xd0] = 0x4001e400
                                            //0x0000004e:    6801        .h      LDR      r1,[r0,#0]
                                            //0x00000050:    2900        .)      CMP      r1,#0
                                            //0x00000052:    d0fc        ..      BEQ      0x4e ; EraseChip + 10
                                            //0x00000054:    2000        .       MOVS     r0,#0
                                            //0x00000056:    4770        pG      BX       lr


                                            //LDR      r0,[pc,#192] ; [0xcc] = 0x4001e504
                                            //MOVS     r0,#1
                                            //ADDS     r1,r1,#0x10
                                            //STR      r0,[r1,#0]
                                            *((uint32_t *)0x4001e514) = 1;

                                            //LDR      r0,[pc,#128] ; [0xd0] = 0x4001e400
                                            //LDR      r1,[r0,#0]
                                            //CMP      r1,#0
                                            //BEQ      0x4e ; EraseChip + 10
                                            while(*((uint32_t *)0x4001e400) == 0);

                                            //MOVS     r0,#0
                                            //BX       lr
                                            //return 0

                                            //UnInit
                                            //0x00000032:    4926        &I      LDR      r1,[pc,#152] ; [0xcc] = 0x4001e504
                                            //0x00000034:    2000        .       MOVS     r0,#0
                                            //0x00000036:    6008        .`      STR      r0,[r1,#0]
                                            //0x00000038:    4825        %H      LDR      r0,[pc,#148] ; [0xd0] = 0x4001e400
                                            //0x0000003a:    6801        .h      LDR      r1,[r0,#0]
                                            //0x0000003c:    2900        .)      CMP      r1,#0
                                            //0x0000003e:    d0fc        ..      BEQ      0x3a ; UnInit + 8
                                            //0x00000040:    2000        .       MOVS     r0,#0
                                            //0x00000042:    4770        pG      BX       lr

                                            //LDR      r1,[pc,#152] ; [0xcc] = 0x4001e504
                                            //MOVS     r0,#0
                                            //STR      r0,[r1,#0]
                                            *((uint32_t *)0x4001e504) = 0;

                                            //LDR      r0,[pc,#148] ; [0xd0] = 0x4001e400
                                            //LDR      r1,[r0,#0]
                                            //CMP      r1,#0
                                            //BEQ      0x3a ; UnInit + 8
                                            while(*((uint32_t *)0x4001e400) == 0);

                                            //MOVS     r0,#0
                                            //BX       lr
                                            //return 0;
                                        }
                                        nrf_delay_ms(1);

                                        uicrPtr = (uint32_t *)NRF_UICR_BASE;
                                        for(i=0; i < check_UART_UICR_item_cnt; i++)
                                        {
                                            if(*uicrPtr++ != 0xffffffff)
                                            {
                                                cPrintLog(CDBG_MAIN_LOG, "UICR erase Error!\n");
                                            }
                                        }

                                        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
                                        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                    
                                        uicrPtr = (uint32_t *)NRF_UICR_BASE;
                                        for(i=0; i < check_UART_UICR_item_cnt; i++)
                                        {
                                            if(check_UART_UICR_buf[i] != 0xffffffff)
                                            {
                                                uicrPtr[i] = check_UART_UICR_buf[i];
                                                while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                            }
                                        }
                                    
                                        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
                                        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                    
                                        if(memcmp(check_UART_UICR_buf, (void *)NRF_UICR_BASE, sizeof(check_UART_UICR_buf)) == 0)
                                        {
                                            cPrintLog(CDBG_MAIN_LOG, "Write Done. Please Reboot.\n");
                                        }
                                        else
                                        {
                                            cPrintLog(CDBG_MAIN_LOG, "Write Error.\n");
                                        }

                                    }
                                    result = true;
                                }
                                else if(check_UART_param_buf[0]=='D')
                                {
                                    APPROTECT_new_val = 0xffffff00;
                                    cPrintLog(CDBG_MAIN_LOG, "Disable DAP. 0x%08x to 0x%08x\n", APPROTECT_old_val, APPROTECT_new_val);
                                    if(APPROTECT_old_val != APPROTECT_new_val)
                                    {
                                        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
                                        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                        NRF_UICR->APPROTECT = APPROTECT_new_val;
                                        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
                                        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
                                        cPrintLog(CDBG_MAIN_LOG, "Write Done. Please Reboot.\n");
                                    }
                                    result = true;
                                }
                            }

                            if(result)
                            {
                                check_UART_send_OK_resp();
                            }
                            else
                            {
                                check_UART_send_ERROR_resp();
                            }
                            check_UART_state = check_UART_wait_cmd_s;
                        }
                        break;

                    default:
                        break;
                }            
            }
            
            (void)nrf_drv_uart_rx(&check_UART_instance, check_UART_RX_buf, 1);  //request isr for read 1byte 
            break;

        case NRF_DRV_UART_EVT_ERROR:
            {
                static int err_cnt = 0;
                if(err_cnt > 5)
                {
                    cPrintLog(CDBG_MAIN_LOG, "UART_Err Exit!\n");
                    check_UART_state = check_UART_parser_exit_s;
                }
            }
            break;
        default:
            break;
    }
}

static check_UART_cmd_processor_result_e check_UART_cmd_processor(void)
{
    check_UART_cmd_processor_result_e ret = check_UART_cmd_processor_boot_r;
    uint32_t err_code;
    int timeout;
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;

#ifdef TEST_FEAUTRE_DFU_SERIAL_TARGET_NRF52_DK
    uart_config.pseltxd   = TX_PIN_NUMBER;
    uart_config.pselrxd   = CHECK_UART_RX_PIN_DEF;
    uart_config.pselcts   = CTS_PIN_NUMBER;
    uart_config.pselrts   = RTS_PIN_NUMBER;
    uart_config.hwfc      = NRF_UART_HWFC_ENABLED;
#else
    uart_config.pseltxd   = PIN_DEF_DTM_TX;
    uart_config.pselrxd   = CHECK_UART_RX_PIN_DEF;
    uart_config.pselcts   = 0xffffffff;
    uart_config.pselrts   = 0xffffffff;
    uart_config.hwfc      = NRF_UART_HWFC_DISABLED;
    uart_config.baudrate  = NRF_UART_BAUDRATE_115200;
#endif

    err_code = nrf_drv_uart_init(&check_UART_instance, &uart_config, check_UART_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        return check_UART_cmd_processor_boot_r;
    }
    nrf_drv_uart_rx_enable(&check_UART_instance);
    err_code = nrf_drv_uart_rx(&check_UART_instance, check_UART_RX_buf, 1);
    if (err_code != NRF_SUCCESS)
    {
#if (bootloader_secure_Developer_logs)
        cPrintLog(CDBG_MAIN_LOG, "Failed initializing rx\n");
#endif
        nrf_drv_uart_uninit(&check_UART_instance);
        return check_UART_cmd_processor_boot_r;
    }

#if (bootloader_secure_Developer_logs)
    if(timeout == 0)
    {
        cPrintLog(CDBG_MAIN_LOG, "Wait Uart Cmd\n");
    }
#endif

    timeout = UART_CMD_WAIT_TICK_CNT;
    do
    {
        if(check_UART_state != check_UART_wait_start_s)
        {
            break;
        }
        nrf_delay_ms(100);
    }while(--timeout);

#if (bootloader_secure_Developer_logs)
    cPrintLog(CDBG_MAIN_LOG, "Timeout Chk:[%d/%d], state:%d\n", timeout, UART_CMD_WAIT_TICK_CNT, check_UART_state);
#endif

    if(timeout)
    {
        cfg_dfu_resource_check_init();
        cfg_dfu_led_ctrl(cfg_dfu_led_ctrl_serial_ctrl_connected);
        while(check_UART_state != check_UART_parser_exit_s)
        {
            switch(check_UART_state)
            {
                case check_UART_cmd_proc_enter_serial_DFU_s:
                    ret = check_UART_cmd_processor_enter_serial_DFU_r;
                    check_UART_state = check_UART_parser_exit_s;
                    break;
                case check_UART_cmd_proc_enter_ble_DFU_s:
                    ret = check_UART_cmd_processor_enter_ble_DFU_r;
                    check_UART_state = check_UART_parser_exit_s;
                    break;
                case check_UART_cmd_proc_reboot_request_s:
                    ret = check_UART_cmd_processor_reboot_request_r;
                    check_UART_state = check_UART_parser_exit_s;
                    break;
                default:
                    //do nothing
                    break;
            }
#if (bootloader_secure_Developer_logs)
            {
                static int old_state = 0;
                if(check_UART_state != old_state)
                {
                    cPrintLog(CDBG_MAIN_LOG, "State %d to %d\n", old_state, check_UART_state);
                    old_state = check_UART_state;
                }
            }
#endif
        }
#if (bootloader_secure_Developer_logs)
        cPrintLog(CDBG_MAIN_LOG, "Exit Cmd ret:%d\n", ret);
#endif
        nrf_delay_ms(10);
    }

    nrf_drv_uart_uninit(&check_UART_instance);
    return ret;
}

bool cfg_bl_dfu_check_enter(int reason /*0:normal, 1:invalid app*/)
{
    bool request_enter_DFU = false;
#if (bootloader_secure_Developer_logs)
    cPrintLog(CDBG_MAIN_LOG, "Uart Chk\n");
#endif
    if(check_UART_connect_with_RX_pin())
    {
        check_UART_cmd_processor_result_e result;
        result = check_UART_cmd_processor();
        switch(result)
        {
            case check_UART_cmd_processor_enter_serial_DFU_r:
                nrf_ble_dfu_enable = false; 
                nrf_serial_dfu_enable = true;
                request_enter_DFU = true;
#if (bootloader_secure_Developer_logs)
                cPrintLog(CDBG_MAIN_LOG, "Enter Serial DFU\n");
#endif
                break;
            case check_UART_cmd_processor_enter_ble_DFU_r:
                nrf_ble_dfu_enable = true; 
                nrf_serial_dfu_enable = false;
                request_enter_DFU = true;
#if (bootloader_secure_Developer_logs)
                cPrintLog(CDBG_MAIN_LOG, "Enter BLE DFU\n");
#endif
                break;
            case check_UART_cmd_processor_reboot_request_r:
                cPrintLog(CDBG_MAIN_LOG, "Reset Req!\n");
                NVIC_SystemReset();
                break;
            default:
                break;
        }
    }
#if (bootloader_secure_Developer_logs)
    cPrintLog(CDBG_MAIN_LOG, "Exit Uart Chk Flag:%d AppErr:%d\n", request_enter_DFU, reason);
#endif
    return request_enter_DFU;
}
#endif
#endif

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
#if defined(FEATURE_WISOL_DEVICE)  && defined(FEATURE_WISOL_BOOTLOADER)
    {
        uint32_t lrReg;
        CFG_GET_LINK_REGISTER(lrReg);
        cPrintLog(CDBG_MAIN_LOG, "app_error_handler! code:%d lr:0x%p %s:%d\n", error_code, (void *)lrReg, p_file_name, line_num);
        (void)lrReg;
    }
#endif
    on_error();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
#if defined(FEATURE_WISOL_DEVICE)  && defined(FEATURE_WISOL_BOOTLOADER)
    {
        uint32_t lrReg;
        CFG_GET_LINK_REGISTER(lrReg);
        cPrintLog(CDBG_MAIN_LOG, "Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x lr:0x%p\n", id, pc, info, (void *)lrReg);
        (void)lrReg;
    }
#endif
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
#if defined(FEATURE_WISOL_DEVICE)  && defined(FEATURE_WISOL_BOOTLOADER)
    {
        uint32_t lrReg;
        CFG_GET_LINK_REGISTER(lrReg);
        cPrintLog(CDBG_MAIN_LOG, "Received an error! code:%d lr:0x%p\n", error_code, (void *)lrReg);
        (void)lrReg;
    }
#endif
    on_error();
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
#if defined(FEATURE_WISOL_DEVICE)  && defined(FEATURE_WISOL_BOOTLOADER)
    cfg_dfu_resource_check_init();
#if (bootloader_secure_Developer_logs)
    cPrintLog(CDBG_MAIN_LOG, "dfu_observer:%d ble:%d, ser:%d\n", evt_type, nrf_ble_dfu_enable, nrf_serial_dfu_enable);
#endif

    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
            cfg_dfu_led_ctrl(cfg_dfu_led_ctrl_error);
            break;
        case NRF_DFU_EVT_DFU_INITIALIZED:
            cfg_dfu_led_ctrl(cfg_dfu_led_ctrl_wait_DFU_download);
            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
            break;
        case NRF_DFU_EVT_DFU_STARTED:
            cfg_dfu_led_ctrl(cfg_dfu_led_ctrl_updating);
            break;
        case NRF_DFU_EVT_DFU_COMPLETED:
            cfg_dfu_resource_check_uninit();
            break;
        default:
            break;
    }
#else
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
        case NRF_DFU_EVT_DFU_INITIALIZED:
            bsp_board_init(BSP_INIT_LEDS);
            bsp_board_led_on(BSP_BOARD_LED_0);
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_off(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
            bsp_board_led_off(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
            break;
        case NRF_DFU_EVT_DFU_STARTED:
            break;
        default:
            break;
    }
#endif
}


/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Inside main");
    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    // Boot the main application.
    nrf_bootloader_app_start();

    // Should never be reached.
    NRF_LOG_INFO("After main");
}


