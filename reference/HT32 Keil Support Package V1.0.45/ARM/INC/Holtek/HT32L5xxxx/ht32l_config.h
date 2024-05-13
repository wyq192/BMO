/*********************************************************************************************************//**
 * @file    ht32l_config.h
 * @version $Rev:: 170          $
 * @date    $Date:: 2024-02-16 #$
 * @brief   Configuration file of HT32.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __HT32L_CONFIG_H
#define __HT32L_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Settings ------------------------------------------------------------------------------------------------*/
#ifdef USE_HT50L3200U_SK
  #define USE_HT32L62141_SK
#endif
#ifdef USE_HT50L3200U
  #define USE_HT32L62141
#endif
#ifdef USE_MEM_HT50L3200U
  #define USE_MEM_HT32L62141
#endif

#ifdef USE_HT32L62141_SK
  #define USE_HT32L52241_SK
#endif
#ifdef USE_HT32L62141
  #define USE_HT32L52231_41
#endif
#ifdef USE_MEM_HT32L62141
  #define USE_MEM_HT32L52241
#endif

#ifdef __cplusplus
}
#endif

#endif
