/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Cor.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __CFG_ENCODE_TX_DATA_H__
#define __CFG_ENCODE_TX_DATA_H__
#include <stdbool.h>
#include <stdint.h>
#include "cfg_config.h"
#ifdef __cplusplus
extern "C" {
#endif

bool check_LatiLongi_dd_Str(const char *str, uint32_t *get_negative_flag, uint32_t *get_integer_number, uint32_t *get_decimal_number);
void make_position_to_dd_Str(uint32_t pos_data, char *str_buf /*max 12 byte*/);
bool make_LatitudeLongitude_to_PosData(const char *latitude_str, const char *longitude_str, uint8_t *pos_buf /*8 bype*/);
bool make_PosData_to_LatitudeLongitude(const uint8_t *pos_buf /*8 bype*/, char *latitude_str /*max size:11(sdd.dddddd\n)*/, char *longitude_str/*max size:12(sddd.dddddd\n)*/);

#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
void encode_12byte_dataV2_NoTracking(uint8_t *buf/*12 byte*/, int event, uint8_t batt_lvl, uint8_t magnet_status);
void encode_12byte_dataV2_GPS(uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint8_t magnet_status);
void encode_12byte_dataV2_WIFI(uint8_t *buf/*12 byte*/, int event, const uint8_t *wifi_mac /*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint8_t magnet_status);
void encode_12byte_dataV2_BLE(uint8_t *buf/*12 byte*/, int event, const uint8_t *ble_mac/*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint8_t magnet_status);
void encode_12byte_dataV2_PosData(int data_type/*main_send_data_type*/, uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint8_t magnet_status);
#else
void encode_12byte_dataV2_NoTracking(uint8_t *buf/*12 byte*/, int event, uint8_t batt_lvl, uint16_t tmp_data);
void encode_12byte_dataV2_GPS(uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint16_t tmp_data);
void encode_12byte_dataV2_WIFI(uint8_t *buf/*12 byte*/, int event, const uint8_t *wifi_mac /*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint16_t tmp_data);
void encode_12byte_dataV2_BLE(uint8_t *buf/*12 byte*/, int event, const uint8_t *ble_mac/*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint16_t tmp_data);
void encode_12byte_dataV2_PosData(int data_type/*main_send_data_type*/, uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint16_t tmp_data);
#endif
void encode_12byte_dataV2_Sensor(uint8_t *buf/*12 byte*/, int event, const void* /*const struct bma_accel_data*/ acc_data, uint32_t als, uint8_t batt_lvl, uint16_t tmp_data);


#ifdef __cplusplus
}
#endif
#endif // __CFG_ENCODE_TX_DATA_H__
