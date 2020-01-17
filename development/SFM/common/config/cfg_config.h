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


#ifndef __CFG_CONFIG_H__
#define __CFG_CONFIG_H__

#if defined ( __CC_ARM )
    #ifndef __WEAK
        #define __WEAK              __weak
    #endif
#elif defined ( __ICCARM__ )
    #ifndef __WEAK
        #define __WEAK              __weak
    #endif
#elif defined   ( __GNUC__ )
    #ifndef __WEAK
        #define __WEAK              __attribute__((weak))
    #endif
#elif defined   ( __TASKING__ )
    #ifndef __WEAK
        #define __WEAK              __attribute__((weak))
    #endif
#endif

#include "cfg_config_defines.h"
#include "cfg_config_setting.h"
#include "cfg_config_app.h"
#include "cfg_util.h"

#include "cfg_board.h"
#endif // __CFG_CONFIG_H__
