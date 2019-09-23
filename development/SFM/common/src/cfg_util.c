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

//util function
void cfg_bin_2_hexadecimal(const uint8_t *pBin, int binSize, char *pHexadecimal)
{
    uint8_t data, dH, dL;
    int i;

    for(i = 0; i < binSize; i++)
    {
        data = *pBin++;
        dH = (data >> 4);
        dL = (data & 0x0F);
        if(dH < 10)
            *pHexadecimal++ = ('0'+ dH);
        else
            *pHexadecimal++ = ('A'+ (dH-10));
        if(dL < 10)
            *pHexadecimal++ = ('0'+ dL);
        else
            *pHexadecimal++ = ('A'+ (dL-10));
    }
    *pHexadecimal = 0;
}

uint32_t cfg_hexadecimal_2_uint_val_big(const char *pHexadecimal, int len)
{
    uint32_t uint_val = 0;
    int i = 0;

    for(i=0; i<len; i++)
    {
        uint_val = (uint_val << 4);
        if(pHexadecimal[i] >= '0' && pHexadecimal[i] <= '9')
        {
            uint_val += (pHexadecimal[i]-'0');
        }
        else if(pHexadecimal[i] >= 'A' && pHexadecimal[i] <= 'F')
        {
            uint_val += ((pHexadecimal[i]-'A')+10);
        }
        else if(pHexadecimal[i] >= 'a' && pHexadecimal[i] <= 'f')
        {
            uint_val += ((pHexadecimal[i]-'a')+10);
        }
    }
    return uint_val;
}

void cfg_hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin)
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

//convert the string to number
int cfg_atoi(const char *str)
{
    int num = 0, i = 0;
    int val = 1;

    if( NULL == str)
    {
        return 0;
    }
    if(str[i]=='-')
    {
        i++;
        val = -1;
    }
    while(str[i] != '\0')
    {
        if(str[i] >= '0' && str[i] <= '9')
        {
            num = num * 10 + (str[i] - '0');
            i++;
        }
        else
        {
            break;
        }
    }
    return (num * val);
}
