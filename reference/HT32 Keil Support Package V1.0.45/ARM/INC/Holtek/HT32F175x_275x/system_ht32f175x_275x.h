/**************************************************************************//**
 * @file    system_ht32f175x_275x.h
 * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File
 *          for the Holtek HT32F175x/275x Device Series
 * @version $Rev:: 486          $
 * @date    $Date:: 2015-12-09 #$
 *
 * @note
 * Copyright (C) Holtek Semiconductor Inc. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) supplies this software for use with Cortex-M processor-based
 * microcontrollers.  This file can be freely distributed within development
 * tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup HT32F175x_275x_system
  * @{
  */


#ifndef __SYSTEM_HT32F175x_275x_H
#define __SYSTEM_HT32F175x_275x_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup HT32F175x_275x_System_Exported_types
  * @{
  */
extern __IO uint32_t SystemCoreClock;  /*!< System Clock Frequency (Core Clock)                             */
/**
  * @}
  */


/** @addtogroup HT32F175x_275x_System_Exported_Functions
  * @{
  */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_HT32F175x_275x_H */


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Holtek Semiconductor Inc. *****END OF FILE***                            */
