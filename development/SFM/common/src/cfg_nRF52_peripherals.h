/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __CFG_NRF52_PERIPHERALS_H__
#define __CFG_NRF52_PERIPHERALS_H__
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
NFC
*******************************************************/
extern bool m_nfc_tag_on;
typedef void (*cfg_nfc_tag_on_callback)(void);
void cfg_nfc_init(cfg_nfc_tag_on_callback callback);
void cfg_nfc_uninit(void);
void cfg_nfc_restart(void);
void cfg_nfc_main_handler(void);

#ifdef __cplusplus
}
#endif
#endif // __CFG_NVM_CTRL_H__
