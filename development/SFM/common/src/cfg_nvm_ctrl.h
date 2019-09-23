/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __CFG_NVM_CTRL_H__
#define __CFG_NVM_CTRL_H__
#include <stdbool.h>
#include <stdint.h>
#include "cfg_config_defines.h"
#ifdef __cplusplus
extern "C" {
#endif
extern bool m_module_parameter_rebuiled_flag;

void module_parameter_update(void);
void module_parameter_check_update(void);  //need set m_module_parameter_update_req
unsigned int module_parameter_get_val(module_parameter_item_e item);
void module_parameter_set_val(module_parameter_item_e item, unsigned int val);
bool module_parameter_get_bootmode(int *bootmode);
void module_parameter_early_read(void);  //just read only for module parameter
bool module_parameter_check_magic(void);

bool registered_mac_update(bool reset_after_update);
bool registered_mac_get_info_wifi(uint32_t *wifi_mac_cnt, uint8_t **wifi_mac_array, uint8_t **wifi_mac_to_position_array);
void registered_mac_remove_all_wifi(void);
bool registered_mac_add_wifi(const uint8_t *wifi_mac, const uint8_t *wifi_position_data/*8byte position data*/);
bool registered_mac_get_info_ble(uint32_t *ble_mac_cnt, uint8_t **ble_mac_array, uint8_t **ble_mac_to_position_array);
void registered_mac_remove_all_ble(void);
bool registered_mac_add_ble(const uint8_t *ble_mac, const uint8_t *ble_position_data/*8byte position data*/);

bool ssid_list_update(void);
void ssid_list_remove_all_white_list(void);
bool ssid_list_add_white_list(const char *ssid_str);
bool ssid_list_remove_white_list(unsigned int index);
void ssid_list_remove_all_black_list(void);
bool ssid_list_add_black_list(const char *ssid_str);
bool ssid_list_remove_black_list(unsigned int index);

void cfg_nvm_init(void);
bool cfg_nvm_factory_reset(bool sys_reset);  //for factory reset
bool cfg_peripheral_device_enable_status_get(peripheral_dev_type peripheral_dev);

#ifdef __cplusplus
}
#endif
#endif // __CFG_NVM_CTRL_H__
