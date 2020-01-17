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


#ifndef __CFG_CONFIG_BOARD_REDEFINE_H__
#define __CFG_CONFIG_BOARD_REDEFINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CDEV_SIGFOX_MONARCH_MODULE //use only monarch module (eg. SRM200A)
#undef SIGFOX_RC_NUMBER_DEFAULT
#define SIGFOX_RC_NUMBER_DEFAULT            1

#undef SIGFOX_SCAN_RC_MODE_DEFAULT
#define SIGFOX_SCAN_RC_MODE_DEFAULT       RC_SCAN_ALWAYS
#endif


#ifdef __cplusplus
}
#endif
#endif // __CFG_CONFIG_BOARD_REDEFINE_H__
