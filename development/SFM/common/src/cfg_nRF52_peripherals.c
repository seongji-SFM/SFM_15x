/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdk_config.h"

#include "nordic_common.h"
#include "app_error.h"
#include "hardfault.h"
#include "nfc_t2t_lib.h"
#include "nfc_ndef_msg.h"
#include "nfc_text_rec.h"

#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_nRF52_peripherals.h"


/******************************************************
NFC
*******************************************************/
bool m_nfc_tag_on = false;
#if !defined (CONFIG_NFCT_PINS_AS_GPIOS)  //it define is preprocessor definitions
#define MAX_REC_COUNT      3     /**< Maximum records count. */
static bool m_nfc_init_flag = false;
static cfg_nfc_tag_on_callback m_nfc_tag_on_CB = NULL;
static uint8_t m_ndef_msg_buf[256];
const uint8_t en_code[] = {'e', 'n'};
const uint8_t no_code[] = {'N', 'O'};
const uint8_t pl_code[] = {'P', 'L'};

static ret_code_t cfg_nfc_text_msg_encode(uint8_t * p_buffer, uint32_t * p_len)
{
    uint32_t             err_code;
    int i, j;
    NFC_NDEF_MSG_DEF(welcome_msg, MAX_REC_COUNT);
    
#if defined(CDEV_SIGFOX_MODULE)
    //make sigfox id
    uint8_t sigfox_id_payload[3/*"ID:"*/ +8/*id size*/ + 1/*null*/] = "ID:";
    cfg_bin_2_hexadecimal(m_module_peripheral_ID.sigfox_device_ID,4,(char*)&sigfox_id_payload[3]);
    NFC_NDEF_TEXT_RECORD_DESC_DEF(sigfox_id_text_rec,
                                  UTF_8,
                                  en_code,
                                  sizeof(en_code),
                                  sigfox_id_payload,
                                  (sizeof(sigfox_id_payload) -1 /*remove null*/));
    err_code = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(welcome_msg), &NFC_NDEF_TEXT_RECORD_DESC(sigfox_id_text_rec));   
    VERIFY_SUCCESS(err_code);

    //make pac code
    uint8_t sigfox_pac_payload[4/*"PAC:"*/ +16/*pack size*/ + 1/*null*/] = "PAC:";
    cfg_bin_2_hexadecimal(m_module_peripheral_ID.sigfox_pac_code,8,(char*)&sigfox_pac_payload[4]);
    NFC_NDEF_TEXT_RECORD_DESC_DEF(sigfox_pac_text_rec,
                                  UTF_8,
                                  pl_code,
                                  sizeof(pl_code),
                                  sigfox_pac_payload,
                                  (sizeof(sigfox_pac_payload) -1 /*remove null*/));
    err_code = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(welcome_msg), &NFC_NDEF_TEXT_RECORD_DESC(sigfox_pac_text_rec));
    VERIFY_SUCCESS(err_code);
#endif
    //make ble mac
    uint8_t ble_mac_payload[17];  //not include '\0'
    uint8_t mac_hex_digit[13];

    cfg_bin_2_hexadecimal(m_module_peripheral_ID.ble_MAC,6,(char*)(char*)mac_hex_digit);
    for(i=0,j=0;i<(sizeof(mac_hex_digit)-1);)
    {
        ble_mac_payload[j++]=mac_hex_digit[i++];
        ble_mac_payload[j++]=mac_hex_digit[i++];
        if(j<sizeof(ble_mac_payload))ble_mac_payload[j++]=':';
    }
    NFC_NDEF_TEXT_RECORD_DESC_DEF(ble_mac_text_rec,
                                  UTF_8,
                                  no_code,
                                  sizeof(no_code),
                                  ble_mac_payload,
                                  sizeof(ble_mac_payload));
    err_code = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(welcome_msg), &NFC_NDEF_TEXT_RECORD_DESC(ble_mac_text_rec));
    VERIFY_SUCCESS(err_code);

    err_code = nfc_ndef_msg_encode(&NFC_NDEF_MSG(welcome_msg), p_buffer, p_len);
    return err_code;
}

static void cfg_nfc_callback(void * p_context, nfc_t2t_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            m_nfc_tag_on = true;
            if(m_nfc_tag_on_CB)m_nfc_tag_on_CB();
            break;

        case NFC_T2T_EVENT_FIELD_OFF:
            break;

        default:
            break;
    }
}

void cfg_nfc_init(cfg_nfc_tag_on_callback callback)
{
    uint32_t err_code;

    if(!m_nfc_init_flag)
    {
        /* Set up NFC */
        err_code = nfc_t2t_setup(cfg_nfc_callback, NULL);
        APP_ERROR_CHECK(err_code);

        /* Provide information about available buffer size to encoding function */
        uint32_t len = sizeof(m_ndef_msg_buf);

        cfg_nfc_text_msg_encode(m_ndef_msg_buf, &len);

        /* Set created message as the NFC payload */
        err_code = nfc_t2t_payload_set((const uint8_t *)m_ndef_msg_buf, len);
        APP_ERROR_CHECK(err_code);
        /* Start sensing NFC field */
        err_code = nfc_t2t_emulation_start();
        m_nfc_init_flag = true;
        m_nfc_tag_on_CB = callback;
        cPrintLog(CDBG_BLE_INFO, "NFC Start\n");
    }
}

void cfg_nfc_uninit(void)
{
    if(m_nfc_init_flag)
    {
        m_nfc_init_flag = false;
        nfc_t2t_emulation_stop();
    }
}

void cfg_nfc_restart(void)
{
    if(m_nfc_init_flag)
    {
        uint32_t len = sizeof(m_ndef_msg_buf);
        nfc_t2t_emulation_stop();
        cfg_nfc_text_msg_encode(m_ndef_msg_buf, &len);
        nfc_t2t_payload_set(m_ndef_msg_buf, len);
        nfc_t2t_emulation_start();
        cPrintLog(CDBG_BLE_INFO, "NFC ReStart\n");
    }
}

void cfg_nfc_main_handler(void)
{
    if(m_nfc_tag_on)
    {
        m_nfc_tag_on = false;
    }
}
#else
void cfg_nfc_init(cfg_nfc_tag_on_callback callback){}
void cfg_nfc_uninit(void){}
void cfg_nfc_restart(void){}
void cfg_nfc_main_handler(void){}
#endif

