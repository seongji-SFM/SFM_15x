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
 * @brief control board device control.
 *
 * This file contains the control wifi modle.
 */


#ifndef __CFG_UTIL_H__
#define __CFG_UTIL_H__
#ifdef __cplusplus
extern "C" {
#endif

//util function//////////////////////////////////////////////////////////////////////////
/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input binary data
 * @param[in]   input binary data size
 * @param[out]  hexadecimal data
 *
 * @return      void
 */
void cfg_bin_2_hexadecimal(const uint8_t *pBin, int binSize, char *pHexadecimal);

/**
 * @brief       Function for hexadecimal (big endian) to integer
 *
 * @param[in]   input hexadecimal data
 * @param[in]   input hexadecimal size (max 8)
 *
 * @return      integer value
 */
uint32_t cfg_hexadecimal_2_uint_val_big(const char *pHexadecimal, int len);

/**
 * @brief       Function for binary to hexadecimal
 *
 * @param[in]   input pHexadecimal data
 * @param[in]   input HexadeccoimalByteCnt data size (byte count)
 * @param[out]  binary data
 *
 * @return      void
 */
void cfg_hexadecimal_2_bin(const char *pHexadecimal, int HexadeccoimalByteCnt, uint8_t *pBin);

/**
 * @brief       Function for string to interger
 *
 * @param[in]   input string
 *
 * @return      interger value
 */
int cfg_atoi(const char *str);
#ifdef __cplusplus
}
#endif
#endif // __WBOARD_CONFIG_DEF__
