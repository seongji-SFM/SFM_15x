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
#include "cfg_encode_tx_data.h"
#include "cfg_config.h"
#include "cfg_dbg_log.h"
#include "cfg_bma250_module.h"

static void make_event_data(uint8_t *buf, int /*main_wakeup_reason_type*/ event, int /*main_send_data_type*/ data_type)
{
#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
    buf[0] = (data_type << 4) | 0x0F;
#else
    buf[0] = (event << 4 ) | (data_type);
#endif
}

static void make_acc_data(uint8_t *buf, const struct bma_accel_data *acc_data)
{
    memset(buf, 0xff, 6);
    if(acc_data)
    {
        buf[0] = (acc_data->x &0XFF00)>>8;
        buf[1] = acc_data->x &0x00FF;
        buf[2] = (acc_data->y &0XFF00)>>8;
        buf[3] = acc_data->y &0x00FF;
        buf[4] = (acc_data->z &0XFF00)>>8;
        buf[5] =  acc_data->z &0x00FF;
    }
}

static void make_tmp_data(uint8_t *buf, uint16_t tmp_data)
{
    volatile int tmp102;
    uint8_t tmp108_a, tmp108_b;

    if(tmp_data >= 0xffff)
    {
        tmp108_a = 0xff;
        tmp108_b = 0xff;
    }
    else
    {
        tmp108_a = (uint8_t)((tmp_data & 0xff00) >> 8);
        tmp108_b = (uint8_t)(tmp_data & 0xff);
    }
    buf[0] = tmp108_a;
    buf[1] = tmp108_b;   
}

/*PosData format*/ //ref cGps_longitude_dddmmtoddd and cGps_longitude_dddmmtoddd
#define LATI_LONGI_STR_CHK_MAX 16
#define LATI_LONGI_DECIMAL_NUMBER_POX_MAX 6
bool check_LatiLongi_dd_Str(const char *str, uint32_t *get_negative_flag, uint32_t *get_integer_number, uint32_t *get_decimal_number)
{
    bool ret = false;
    bool check_null = false;
    bool decimal_point_flag = false;
    
    uint32_t negative_flag = 0, integer_number = 0, decimal_number = 0;
    int i, integer_cnt = 0, decimal_cnt = 0;

    if(str[0] == '-')
    {
        negative_flag = 1;
        i = 1;
    }
    else 
    {
        i = 0;
    }
    
    for(; i < LATI_LONGI_STR_CHK_MAX; i++)
    {
        if(str[i] == '\0')  //null
        {
            check_null = true;
        }
        else if(str[i] == '.')
        {
            if(decimal_point_flag)
            {
                break;
            }
            else
            {
                decimal_point_flag = 1;
            }
        }
        else if(str[i] >= '0' && str[i] <= '9')
        {
            if(decimal_point_flag)
            {
                if(++decimal_cnt <= LATI_LONGI_DECIMAL_NUMBER_POX_MAX)
                {
                    decimal_number = ((decimal_number *10)+(str[i]-'0'));
                }
            }
            else
            {
                if(++integer_cnt <= 3)  //Latitude max 90, Longitude max 180
                {
                    integer_number = ((integer_number *10)+(str[i]-'0'));
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            break;
        }
    }
    if(check_null && i > 0)
    {
        ret = true;
        if(get_negative_flag)*get_negative_flag=negative_flag;
        if(get_integer_number)*get_integer_number=integer_number;
        if(get_decimal_number)*get_decimal_number=decimal_number;
    }
    return ret;
}

void make_position_to_dd_Str(uint32_t pos_data, char *str_buf /*max 12 byte*/)
{
    char pos_str_buf[LATI_LONGI_STR_CHK_MAX];
    uint32_t negative_flag = 0, integer_number = 0, decimal_number = 0;
    int str_idx;

    if(pos_data & 0x80000000)
    {
        negative_flag = 1;
        pos_data = (pos_data - 0x80000000);

    }

    if(pos_data >= 1000000)
    {
        integer_number = (pos_data / 1000000);
        pos_data = pos_data - (integer_number*1000000);
    }
    decimal_number = pos_data;

    if(negative_flag)
    {
        pos_str_buf[0] = '-';
        str_idx = 1;
    }
    else
    {
        str_idx = 0;
    }

    snprintf(&(pos_str_buf[str_idx]), (sizeof(pos_str_buf)-1), "%d.%06d", integer_number, decimal_number);

    if(str_buf)
    {
        strncpy(str_buf, pos_str_buf, 11);
        str_buf[11]='\0';
    }
}

bool make_LatitudeLongitude_to_PosData(const char *latitude_str, const char *longitude_str, uint8_t *pos_buf /*8 bype*/)
{
    bool ret = false;
    uint32_t latitude_work, latitude_negative_flag = 0, latitude_integer_number = 0, latitude_decimal_number = 0;
    uint32_t longitude_work, longitude_negative_flag = 0, longitude_integer_number = 0, longitude_decimal_number = 0;
        
    if(check_LatiLongi_dd_Str(latitude_str, &latitude_negative_flag, &latitude_integer_number, &latitude_decimal_number))
    {
        if(check_LatiLongi_dd_Str(longitude_str, &longitude_negative_flag, &longitude_integer_number, &longitude_decimal_number))
        {
            if(latitude_integer_number < 90 && longitude_integer_number < 180)
            {
                ret = true;
                if(pos_buf)  //ref cGps_latitude_ddmmtoddd and cGps_longitude_dddmmtoddd
                {
                    latitude_work = ((latitude_integer_number*1000000) + latitude_decimal_number);
                    if(latitude_negative_flag)
                    {
                        latitude_work = (latitude_work | 0x80000000);
                    }
                    longitude_work = ((longitude_integer_number*1000000) + longitude_decimal_number);
                    if(longitude_negative_flag)
                    {
                        longitude_work = (longitude_work | 0x80000000);
                    }

                    //encode latitude
                    pos_buf[0] = (uint8_t)((latitude_work >> 24) & 0xFF);
                    pos_buf[1] = (uint8_t)((latitude_work >> 16) & 0xFF);
                    pos_buf[2] = (uint8_t)((latitude_work >> 8) & 0xFF);
                    pos_buf[3] = (uint8_t)(latitude_work & 0xFF);

                    //encode longitude
                    pos_buf[4] = (uint8_t)((longitude_work >> 24) & 0xFF);
                    pos_buf[5] = (uint8_t)((longitude_work >> 16) & 0xFF);
                    pos_buf[6] = (uint8_t)((longitude_work >> 8) & 0xFF);
                    pos_buf[7] = (uint8_t)(longitude_work & 0xFF);
                }
            }
        }
    }
    return ret;
}

bool make_PosData_to_LatitudeLongitude(const uint8_t *pos_buf /*8 bype*/, char *latitude_str /*max size:11(sdd.dddddd\n)*/, char *longitude_str/*max size:12(sddd.dddddd\n)*/)
{   
    uint32_t latitude_work, longitude_work;
    char latitude_work_buf[LATI_LONGI_STR_CHK_MAX], longitude_work_buf[LATI_LONGI_STR_CHK_MAX];
    
    if(!pos_buf)return false;

    latitude_work = ((((uint32_t)(pos_buf[0])) << 24) | (((uint32_t)(pos_buf[1])) << 16) | (((uint32_t)(pos_buf[2])) << 8) | (((uint32_t)(pos_buf[3]))));
    longitude_work = ((((uint32_t)(pos_buf[4])) << 24) | (((uint32_t)(pos_buf[5])) << 16) | (((uint32_t)(pos_buf[6])) << 8) | (((uint32_t)(pos_buf[7]))));

    make_position_to_dd_Str(latitude_work, latitude_work_buf);
    make_position_to_dd_Str(longitude_work, longitude_work_buf);
    if(latitude_str)
    {
        strncpy(latitude_str, latitude_work_buf, 11);
        latitude_str[11]='\0';
    }
    if(longitude_str)
    {
        strncpy(longitude_str, longitude_work_buf, 12);
        longitude_str[12]='\0';
    }
    return true;
}

static void make_als_data(uint8_t * buf, uint32_t als)
{
    //als data max 0x8000 * 40 : about 1,310,720 (19bit data)
    uint32_t als_convert;
    uint16_t als_en_data;

    if(als >= 0xffff)
    {
        als_convert = 0xffff;
    }
    else
    {
        if(als > 0xff00)
        {
            als_convert = 0xff00;
        }
        else
        {
            als_convert = als;
        }
    }
    als_en_data = (uint16_t)(als_convert);
    buf[0] = (uint8_t)(als_en_data >> 8);
    buf[1] = (uint8_t)(als_en_data & 0xFF);
}

#if (CFG_UPLOAD_DATA_FORMAT_TYPE == NEW_SIGFOX_DATA_FORMAT_V2)
void encode_12byte_dataV2_NoTracking(uint8_t *buf/*12 byte*/, int event, uint8_t batt_lvl, uint8_t magnet_status)
{
    make_event_data(&buf[0], event, main_send_data_none);
    memset(&buf[1], 0xff, 8);
    buf[8] = 0xff;
    buf[9] = batt_lvl;
    buf[10] = magnet_status;
    buf[11] = 0xff;
}

void encode_12byte_dataV2_PosData(int data_type/*main_send_data_type*/, uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint8_t magnet_status)
{
    make_event_data(&buf[0], event, data_type);
    if(gps_data)
    {
        memcpy(&buf[1], gps_data, 8);
    }
    else
    {
        memset(&buf[1], 0xff, 8);
    }
    buf[9] = batt_lvl;
    buf[10] = magnet_status;
    buf[11] = 0xff;
}

void encode_12byte_dataV2_GPS(uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint8_t magnet_status)
{
    make_event_data(&buf[0], event, main_send_data_gps);
    if(gps_data)
    {
        memcpy(&buf[1], gps_data, 8);
    }
    else
    {
        memset(&buf[1], 0xff, 8);
    }
    buf[9] = batt_lvl;
    buf[10] = magnet_status;
    buf[11] = 0xff;
}

void encode_12byte_dataV2_WIFI(uint8_t *buf/*12 byte*/, int event, const uint8_t *wifi_mac /*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint8_t magnet_status)
{
    make_event_data(&buf[0], event, main_send_data_wifi);
    if(wifi_mac)
    {
        memcpy(&buf[1], wifi_mac, 6);
    }
    else
    {
        memset(&buf[1], 0xff, 6);
    }
    buf[7] = (uint8_t)rssi;
    buf[8] = 0xff;
    buf[9] = batt_lvl;
    buf[10] = magnet_status;
    buf[11] = 0xff;
}

void encode_12byte_dataV2_BLE(uint8_t *buf/*12 byte*/, int event, const uint8_t *ble_mac/*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint8_t magnet_status)
{
    make_event_data(&buf[0], event, main_send_data_ble);
    if(ble_mac)
    {
        memcpy(&buf[1], ble_mac, 6);
    }
    else
    {
        memset(&buf[1], 0xff, 6);
    }
    buf[7] = (uint8_t)rssi;   
    buf[8] = 0xff;
    buf[9] = batt_lvl;
    buf[10] = magnet_status;
    buf[11] = 0xff;
}
#else
void encode_12byte_dataV2_NoTracking(uint8_t *buf/*12 byte*/, int event, uint8_t batt_lvl, uint16_t  tmp_data)
{
    make_event_data(&buf[0], event, main_send_data_none);
    memset(&buf[1], 0xff, 8);
    buf[9] = batt_lvl;
    make_tmp_data(&buf[10], tmp_data);
}

void encode_12byte_dataV2_PosData(int data_type/*main_send_data_type*/, uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint16_t tmp_data)
{
    make_event_data(&buf[0], event, data_type);
    if(gps_data)
    {
        memcpy(&buf[1], gps_data, 8);
    }
    else
    {
        memset(&buf[1], 0xff, 8);
    }
    buf[9] = batt_lvl;
    make_tmp_data(&buf[10], tmp_data);
}

void encode_12byte_dataV2_GPS(uint8_t *buf/*12 byte*/, int event, const uint8_t *gps_data /*8 byte*/,uint8_t batt_lvl, uint16_t tmp_data)
{
    make_event_data(&buf[0], event, main_send_data_gps);
    if(gps_data)
    {
        memcpy(&buf[1], gps_data, 8);
    }
    else
    {
        memset(&buf[1], 0xff, 8);
    }
    buf[9] = batt_lvl;
    make_tmp_data(&buf[10], tmp_data);
}

void encode_12byte_dataV2_WIFI(uint8_t *buf/*12 byte*/, int event, const uint8_t *wifi_mac /*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint16_t tmp_data)
{
    make_event_data(&buf[0], event, main_send_data_wifi);
    if(wifi_mac)
    {
        memcpy(&buf[1], wifi_mac, 6);
    }
    else
    {
        memset(&buf[1], 0xff, 6);
    }
    buf[7] = (uint8_t)rssi;
    buf[8] = 0xff;
    buf[9] = batt_lvl;
	
    make_tmp_data(&buf[10], tmp_data);
}

void encode_12byte_dataV2_BLE(uint8_t *buf/*12 byte*/, int event, const uint8_t *ble_mac/*6 byte*/,int8_t rssi, uint8_t batt_lvl, uint16_t tmp_data)
{
    make_event_data(&buf[0], event, main_send_data_ble);
    if(ble_mac)
    {
        memcpy(&buf[1], ble_mac, 6);
    }
    else
    {
        memset(&buf[1], 0xff, 6);
    }
    buf[7] = (uint8_t)rssi;   
    buf[8] = 0xff;
    buf[9] = batt_lvl;
    make_tmp_data(&buf[10], tmp_data);
}
#endif

void encode_12byte_dataV2_Sensor(uint8_t *buf/*12 byte*/, int event, const /*struct bma_accel_data*/ void* acc_data, uint32_t als, uint8_t batt_lvl, uint16_t tmp_data)
{
    const struct bma_accel_data* p_acc_data = (const struct bma_accel_data*)acc_data;
    make_event_data(&buf[0], event, main_send_data_sensor);
    buf[9] = batt_lvl;
    make_acc_data(&buf[1], acc_data);
    make_als_data(&buf[7], als);
    make_tmp_data(&buf[10], tmp_data);
}

