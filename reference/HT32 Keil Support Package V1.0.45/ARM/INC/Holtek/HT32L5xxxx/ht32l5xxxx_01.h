/***************************************************************************//**
 * @file    ht32l5xxxx_01.h
 * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer Header File
 * @version $Rev:: 170          $
 * @date    $Date:: 2024-02-16 #$
 *
 * @note
 * Copyright (C) Holtek Semiconductor Inc. All rights reserved.
 *
 * @par
 * Holtek supplies this software for Cortex-M processor-based
 * microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such ARM-based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

// Supported Device
// ========================================
//   HT32L52231, HT32L52241

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup HT32L5xxxx HT32L5xxxx
  * @{
  */


#ifndef __HT32L5XXXX_01_H__
#define __HT32L5XXXX_01_H__

#include "ht32l_config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#if !defined(USE_HT32L52231_41)

  //#define USE_HT32L52231_41

#endif

#if !defined(USE_NOCHIP) && \
    !defined(USE_HT32L52231_41)

  #error Please add "USE_HT32Fxxxxx_xx" define into C Preprocessor Symbols of the Project configuration.

#endif

/** @addtogroup Library_configuration_section
  * @{
  */
/**
 * @brief Value of the High Speed Internal oscillator in Hz
  */
#define HSI_VALUE      8000000UL   /*!< Value of the High Speed Internal oscillator in Hz                 */


/**
 * @brief Value of the Low Speed Internal oscillator in Hz
  */
#define LSI_VALUE         32000UL     /*!< Value of the Low Speed Internal oscillator in Hz                 */


/**
 * @brief Value of the Low Speed External oscillator in Hz
  */
#define LSE_VALUE         32768UL     /*!< Value of the Low Speed External oscillator in Hz                 */


/**
 * @brief Adjust the High Speed External oscillator (HSE) Startup Timeout value
   */
#define HSE_READY_TIME    ((uint16_t)0xFFFF)    /*!< Time out for HSE start up                              */
/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
#define __MPU_PRESENT             0    /*!< MPU present or not                                              */
#define __VTOR_PRESENT            1    /*!< VTOR present or not                                             */
#define __NVIC_PRIO_BITS          2    /*!< Number of Bits used for Priority Levels                         */
#define __Vendor_SysTickConfig    0    /*!< Set to 1 if different SysTick Config is used                    */

/**
  * @}
  */


/** @addtogroup Configuration_for_Interrupt_Number
  * @{
  */
typedef enum IRQn
{
/****** Cortex-M0+ Processor Exceptions Numbers ********************************                            */
  NonMaskableInt_IRQn     = -14,    /*!< 2 Non Maskable Interrupt                                           */
  HardFault_IRQn          = -13,    /*!< 3 Cortex-M0+ Hard Fault Interrupt                                  */
  SVCall_IRQn             = -5,     /*!< 11 Cortex-M0+ SV Call Interrupt                                    */
  PendSV_IRQn             = -2,     /*!< 14 Cortex-M0+ Pend SV Interrupt                                    */
  SysTick_IRQn            = -1,     /*!< 15 Cortex-M0+ System Tick Interrupt                                */

/******  HT32 Specific Interrupt Numbers ***************************************                            */
  LVD_BOD_IRQn            = 0,      /*!< Low voltage & Brown-out detection interrupt                        */
  RTC_IRQn                = 1,      /*!< RTC Wake-up Interrupt                                              */
  FLASH_IRQn              = 2,      /*!< FLASH global Interrupt                                             */
  EVWUP_IRQn              = 3,      /*!< EXTI Event Wake-up & WAKEUP pin Interrupt                          */
  EXTI0_1_IRQn            = 4,      /*!< EXTI0-1 Line detection Interrupt                                   */
  EXTI2_3_IRQn            = 5,      /*!< EXTI2-3 Line detection Interrupt                                   */
  EXTI4_15_IRQn           = 6,      /*!< EXTI4-15 Line detection Interrupt                                  */
  ADC0_IRQn               = 8,      /*!< ADC Interrupt                                                      */
  MCTM0_IRQn              = 10,     /*!< Motor Control Timer interrupt                                      */
  GPTM0_IRQn              = 12,     /*!< General-Purpose Timer Interrupt                                    */
  SCTM0_IRQn              = 13,     /*!< Single Channel Timer0 Interrupt                                    */
  SCTM1_IRQn              = 14,     /*!< Single Channel Timer1 Interrupt                                    */
  BFTM0_IRQn              = 17,     /*!< Basic Function Timer0 Interrupt                                    */
  BFTM1_IRQn              = 18,     /*!< Basic Function Timer1 Interrupt                                    */
  I2C0_IRQn               = 19,     /*!< I2C0 global Interrupt                                              */
  I2C1_IRQn               = 20,     /*!< I2C1 global Interrupt                                              */
  SPI0_IRQn               = 21,     /*!< SPI0 global Interrupt                                              */
  SPI1_IRQn               = 22,     /*!< SPI1 global Interrupt                                              */
  USART0_IRQn             = 23,     /*!< USART global Interrupt                                             */
  UART0_IRQn              = 25,     /*!< UART0 global Interrupt                                             */
  UART1_IRQn              = 26,     /*!< UART1 global Interrupt                                             */
  PDMACH0_1_IRQn          = 30,     /*!< PDMA channel 0-1 interrupt                                         */
  PDMACH2_5_IRQn          = 31,     /*!< PDMA channel 2-5 interrupt                                         */
} IRQn_Type;

#define EXTI0_IRQn EXTI0_1_IRQn
#define EXTI1_IRQn EXTI0_1_IRQn
#define EXTI2_IRQn EXTI2_3_IRQn
#define EXTI3_IRQn EXTI2_3_IRQn
#define EXTI4_IRQn EXTI4_15_IRQn
#define EXTI5_IRQn EXTI4_15_IRQn
#define EXTI6_IRQn EXTI4_15_IRQn
#define EXTI7_IRQn EXTI4_15_IRQn
#define EXTI8_IRQn EXTI4_15_IRQn
#define EXTI9_IRQn EXTI4_15_IRQn
#define EXTI10_IRQn EXTI4_15_IRQn
#define EXTI11_IRQn EXTI4_15_IRQn
#define EXTI12_IRQn EXTI4_15_IRQn
#define EXTI13_IRQn EXTI4_15_IRQn
#define EXTI14_IRQn EXTI4_15_IRQn
#define EXTI15_IRQn EXTI4_15_IRQn

#define PDMACH0_IRQn PDMACH0_1_IRQn
#define PDMACH1_IRQn PDMACH0_1_IRQn
#define PDMACH2_IRQn PDMACH2_5_IRQn
#define PDMACH3_IRQn PDMACH2_5_IRQn
#define PDMACH4_IRQn PDMACH2_5_IRQn
#define PDMACH5_IRQn PDMACH2_5_IRQn

/**
  * @}
  */

#include "core_cm0plus.h"              /* Cortex-M0+ processor and core peripherals                         */
#include "system_ht32l5xxxx_01.h"      /* HT32 system                                                       */


/** @addtogroup Exported_Types
  * @{
  */

typedef signed long long s64;
typedef signed int     s32;
typedef signed short   s16;
typedef signed char    s8;

typedef const s64      sc64;          /*!< Read Only                                                        */
typedef const s32      sc32;          /*!< Read Only                                                        */
typedef const s16      sc16;          /*!< Read Only                                                        */
typedef const s8       sc8;           /*!< Read Only                                                        */

typedef __IO s64       vs64;
typedef __IO s32       vs32;
typedef __IO s16       vs16;
typedef __IO s8        vs8;

typedef __I s64        vsc64;         /*!< Read Only                                                        */
typedef __I s32        vsc32;         /*!< Read Only                                                        */
typedef __I s16        vsc16;         /*!< Read Only                                                        */
typedef __I s8         vsc8;          /*!< Read Only                                                        */

typedef unsigned long long u64;
typedef unsigned int   u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef const u64      uc64;          /*!< Read Only                                                        */
typedef const u32      uc32;          /*!< Read Only                                                        */
typedef const u16      uc16;          /*!< Read Only                                                        */
typedef const u8       uc8;           /*!< Read Only                                                        */

typedef __IO u64       vu64;
typedef __IO u32       vu32;
typedef __IO u16       vu16;
typedef __IO u8        vu8;

typedef __I u64        vuc64;         /*!< Read Only                                                        */
typedef __I u32        vuc32;         /*!< Read Only                                                        */
typedef __I u16        vuc16;         /*!< Read Only                                                        */
typedef __I u8         vuc8;          /*!< Read Only                                                        */


typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;

#if !defined(bool) && !defined(__cplusplus) // user may already included <stdbool.h> or CPP
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#define false FALSE
#define true  TRUE
#else
#define FALSE 0
#define TRUE  1
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus;

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/**
  * @}
  */

#if defined (__CC_ARM)
  #define __ALIGN4 __align(4)
#elif defined (__ICCARM__)
  #define __ALIGN4 _Pragma("data_alignment = 4")
#elif defined (__GNUC__)
  #define __ALIGN4  __attribute__((aligned(4)))
#endif

#if defined (__GNUC__)
  #define __PACKED_H
  #define __PACKED_F __attribute__ ((packed))
#elif defined (__ICCARM__) || (__CC_ARM)
  #define __PACKED_H __packed
  #define __PACKED_F
#endif

#if defined (__CC_ARM)
#pragma anon_unions
#endif


#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)


/**
 * @brief Exported constants and macro
 */
#define IS_CONTROL_STATUS(STATUS) ((STATUS == DISABLE) || (STATUS == ENABLE))

#define wb(addr, value)     (*((u8  volatile *) (addr)) = value)
#define rb(addr)            (*((u8  volatile *) (addr)))
#define whw(addr, value)    (*((u16 volatile *) (addr)) = value)
#define rhw(addr)           (*((u16 volatile *) (addr)))
#define ww(addr, value)     (*((u32 volatile *) (addr)) = value)
#define rw(addr)            (*((u32 volatile *) (addr)))


#define ResetBit_BB(Addr, BitNumber)        (rw(Addr) &= ~(1UL << BitNumber))
#define SetBit_BB(Addr, BitNumber)          (rw(Addr) |= (1UL << BitNumber))
#define GetBit_BB(Addr, BitNumber)          ((rw(Addr) >> BitNumber) & 1UL)
#define WriteBit_BB(Addr, BitNumber, Value) (Addr = ((Addr & ~((u32)1 << BitNumber)) | ((u32)Value << BitNumber)))

#define STRCAT2_(a, b)       a##b
#define STRCAT2(a, b)        STRCAT2_(a, b)
#define STRCAT3_(a, b, c)    a##b##c
#define STRCAT3(a, b, c)     STRCAT3_(a, b, c)

#define IPN_NULL                        (0)
#define IPN_MCTM0                       (0x4002C000)
#define IPN_MCTM1                       (0x4002D000)
#define IPN_GPTM0                       (0x4006E000)
#define IPN_GPTM1                       (0x4006F000)
#define IPN_SCTM0                       (0x40034000)
#define IPN_SCTM1                       (0x40074000)
#define IPN_SCTM2                       (0x40035000)
#define IPN_SCTM3                       (0x40075000)
#define IPN_PWM0                        (0x40031000)
#define IPN_PWM1                        (0x40071000)
#define IPN_PWM2                        (0x40031000)
#define IPN_BFTM0                       (0x40076000)
#define IPN_BFTM1                       (0x40077000)
#define IPN_CHECK(IP)                   STRCAT2(IPN_, IP)
#define IS_IPN_BFTM(IP)                 (IPN_CHECK(IP) == IPN_BFTM0) || (IPN_CHECK(IP) == IPN_BFTM1)
#define IS_IPN_MCTM(IP)                 (IPN_CHECK(IP) == IPN_MCTM0) || (IPN_CHECK(IP) == IPN_MCTM1)
#define IS_IPN_GPTM(IP)                 (IPN_CHECK(IP) == IPN_GPTM0) || (IPN_CHECK(IP) == IPN_GPTM1)
#define IS_IPN_SCTM(IP)                 (IPN_CHECK(IP) == IPN_SCTM0) || (IPN_CHECK(IP) == IPN_SCTM1) || (IPN_CHECK(IP) == IPN_SCTM2) || (IPN_CHECK(IP) == IPN_SCTM3)
#define IS_IPN_PWM(IP)                  (IPN_CHECK(IP) == IPN_PWM0) || (IPN_CHECK(IP) == IPN_PWM1) || (IPN_CHECK(IP) == IPN_PWM2)
#define IS_IPN_TM(IP)                   (IS_IPN_MCTM(IP) || IS_IPN_GPTM(IP) || IS_IPN_SCTM(IP) || IS_IPN_PWM(IP))


/** @addtogroup Peripheral_Registers_Structures
  * @{
  */


/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */
typedef struct
{
                                 /* USART0: 0x40000000                                                      */
                                 /* USART1: 0x40040000                                                      */
                                 /* UART0:  0x40001000                                                      */
                                 /* UART2:  0x40002000                                                      */
                                 /* UART1:  0x40041000                                                      */
                                 /* UART3:  0x40042000                                                      */
  __IO uint32_t DR;              /*!< 0x000         Data Register                                           */
  __IO uint32_t CR;              /*!< 0x004         Control Register                                        */
  __IO uint32_t FCR;             /*!< 0x008         FIFO Control Register                                   */
  __IO uint32_t IER;             /*!< 0x00C         Interrupt Enable Register                               */
  __IO uint32_t SR;              /*!< 0x010         Status Register                                         */
  __IO uint32_t TPR;             /*!< 0x014         Timing Parameter Register                               */
  __IO uint32_t ICR;             /*!< 0x018         IrDA COntrol Register                                   */
  __IO uint32_t RCR;             /*!< 0x01C         RS485 Control Register                                  */
  __IO uint32_t SCR;             /*!< 0x020         Synchronous Control Register                            */
  __IO uint32_t DLR;             /*!< 0x024         Divisor Latch Register                                  */
  __IO uint32_t DTR;             /*!< 0x028         Debug/Test Register                                     */
} HT_USART_TypeDef;


/**
 * @brief SPI
 */
typedef struct
{
                                 /* SPI0: 0x40004000                                                        */
                                 /* SPI1: 0x40044000                                                        */
  __IO uint32_t CR0;             /*!< 0x000         Control Register 0                                      */
  __IO uint32_t CR1;             /*!< 0x004         Control Register 1                                      */
  __IO uint32_t IER;             /*!< 0x008         Interrupt Enable Register                               */
  __IO uint32_t CPR;             /*!< 0x00C         Clock Prescaler Register                                */
  __IO uint32_t DR;              /*!< 0x010         Data Register                                           */
  __IO uint32_t SR;              /*!< 0x014         Status Register                                         */
  __IO uint32_t FCR;             /*!< 0x018         FIFO Control Register                                   */
  __IO uint32_t FSR;             /*!< 0x01C         FIFO Status Register                                    */
  __IO uint32_t FTOCR;           /*!< 0x020         FIFO Time Out Counter Register                          */
} HT_SPI_TypeDef;


/**
 * @brief Analog to Digital Converter
 */
typedef struct
{
                                 /* ADC: 0x40010000                                                         */
  __IO uint32_t CR;              /*!< 0x000         ADC Conversion Control Register                         */
  __IO uint32_t LST[2];          /*!< 0x004 - 0x008 ADC Conversion List Register 0-1                        */
       uint32_t RESERVE0[5];     /*!< 0x00C - 0x01C Reserved                                                */
  __IO uint32_t STR;             /*!< 0x020         ADC Input Sampling Time Register                        */
       uint32_t RESERVE1[3];     /*!< 0x024 - 0x02C Reserved                                                */
  __IO uint32_t DR[8];           /*!< 0x030 - 0x04C ADC Conversion Data Register 0-7                        */
       uint32_t RESERVE2[8];     /*!< 0x050 - 0x06C Reserved                                                */
  __IO uint32_t TCR;             /*!< 0x070         ADC Trigger Control Register                            */
  __IO uint32_t TSR;             /*!< 0x074         ADC Trigger Source Register                             */
  __IO uint32_t WCR;             /*!< 0x078         ADC Watchdog Control Register                           */
  __IO uint32_t WTR;             /*!< 0x07C         ADC Watchdog Threshold Register                         */
  __IO uint32_t IER;             /*!< 0x080         ADC Interrupt Enable Register                           */
  __IO uint32_t IRAW;            /*!< 0x084         ADC Interrupt Raw Status Register                       */
  __IO uint32_t ISR;             /*!< 0x088         ADC Interrupt Status Register                           */
  __IO uint32_t ICLR;            /*!< 0x08C         ADC Interrupt Clear Register                            */
  __IO uint32_t PDMAR;           /*!< 0x090         ADC PDMA Request Register                               */
       uint32_t RESERVE3[3];     /*!< 0x094 - 0x9C  Reserved                                                */
  __IO uint32_t VREFCR;          /*!< 0x0A0         Reference Voltage Control Register                      */
  __IO uint32_t VREFVALR;        /*!< 0x0A4         Reference Voltage Value Register                        */
       uint32_t RESERVE4[2];     /*!< 0x0A8 - 0xAC  Reserved                                                */
  __IO uint32_t TSCR;            /*!< 0x0B0         Temperature Sensor Control Register                     */
  __IO uint32_t TSCAL0R;         /*!< 0x0B4         Temperature Sensor Value Register                       */
  __IO uint32_t TSCAL1R;         /*!< 0x0B8         Temperature Sensor Value Register                       */
} HT_ADC_TypeDef;


/**
 * @brief General Purpose I/O
 */
typedef struct
{
                                 /* GPIOA: 0x400B0000                                                       */
                                 /* GPIOB: 0x400B2000                                                       */
                                 /* GPIOC: 0x400B4000                                                       */
                                 /* GPIOD: 0x400B6000                                                       */
                                 /* GPIOE: 0x400B8000                                                       */
                                 /* GPIOF: 0x400BA000                                                       */
  __IO uint32_t DIRCR;           /*!< 0x000         Data Direction Control Register                         */
  __IO uint32_t INER;            /*!< 0x004         Input function enable register                          */
  __IO uint32_t PUR;             /*!< 0x008         Pull-Up Selection Register                              */
  __IO uint32_t PDR;             /*!< 0x00C         Pull-Down Selection Register                            */
  __IO uint32_t ODR;             /*!< 0x010         Open Drain Selection Register                           */
  __IO uint32_t DRVR;            /*!< 0x014         Drive Current Selection Register                        */
  __IO uint32_t LOCKR;           /*!< 0x018         Lock Register                                           */
  __IO uint32_t DINR;            /*!< 0x01c         Data Input Register                                     */
  __IO uint32_t DOUTR;           /*!< 0x020         Data Output Register                                    */
  __IO uint32_t SRR;             /*!< 0x024         Output Set and Reset Control Register                   */
  __IO uint32_t RR;              /*!< 0x028         Output Reset Control Register                           */
} HT_GPIO_TypeDef;


/**
 * @brief AFIO
 */
typedef struct
{
                                 /* AFIO: 0x40022000                                                        */
  __IO uint32_t ESSR[2];         /*!< 0x000         EXTI Source Selection Register 0 ~ 1                    */
       uint32_t RESERVE0[6];     /*!< 0x008 - 0x01C Reserved                                                */
  __IO uint32_t GPACFGR[2];      /*!< 0x020         GPIO Port A Configuration Register 0 ~ 1                */
  __IO uint32_t GPBCFGR[2];      /*!< 0x028         GPIO Port B Configuration Register 0 ~ 1                */
  __IO uint32_t GPCCFGR[2];      /*!< 0x030         GPIO Port C Configuration Register 0 ~ 1                */
} HT_AFIO_TypeDef;


/**
 * @brief External Interrupt/Event Controller
 */
typedef struct
{
                                 /* EXTI: 0x40024000                                                        */
  __IO uint32_t CFGR0;           /*!< 0x000         EXTI Interrupt 0 Configuration Register                 */
  __IO uint32_t CFGR1;           /*!< 0x004         EXTI Interrupt 1 Configuration Register                 */
  __IO uint32_t CFGR2;           /*!< 0x008         EXTI Interrupt 2 Configuration Register                 */
  __IO uint32_t CFGR3;           /*!< 0x00C         EXTI Interrupt 3 Configuration Register                 */
  __IO uint32_t CFGR4;           /*!< 0x010         EXTI Interrupt 4 Configuration Register                 */
  __IO uint32_t CFGR5;           /*!< 0x014         EXTI Interrupt 5 Configuration Register                 */
  __IO uint32_t CFGR6;           /*!< 0x018         EXTI Interrupt 6 Configuration Register                 */
  __IO uint32_t CFGR7;           /*!< 0x01C         EXTI Interrupt 7 Configuration Register                 */
  __IO uint32_t CFGR8;           /*!< 0x020         EXTI Interrupt 8 Configuration Register                 */
  __IO uint32_t CFGR9;           /*!< 0x024         EXTI Interrupt 9 Configuration Register                 */
  __IO uint32_t CFGR10;          /*!< 0x028         EXTI Interrupt 10 Configuration Register                */
  __IO uint32_t CFGR11;          /*!< 0x02C         EXTI Interrupt 11 Configuration Register                */
  __IO uint32_t CFGR12;          /*!< 0x030         EXTI Interrupt 12 Configuration Register                */
  __IO uint32_t CFGR13;          /*!< 0x034         EXTI Interrupt 13 Configuration Register                */
  __IO uint32_t CFGR14;          /*!< 0x038         EXTI Interrupt 14 Configuration Register                */
  __IO uint32_t CFGR15;          /*!< 0x03C         EXTI Interrupt 15 Configuration Register                */
  __IO uint32_t CR;              /*!< 0x040         EXTI Interrupt Control Register                         */
  __IO uint32_t EDGEFLGR;        /*!< 0x044         EXTI Interrupt Edge Flag Register                       */
  __IO uint32_t EDGESR;          /*!< 0x048         EXTI Interrupt Edge Status Register                     */
  __IO uint32_t SSCR;            /*!< 0x04C         EXTI Interrupt Software Set Command Register            */
  __IO uint32_t WAKUPCR;         /*!< 0x050         EXTI Interrupt Wakeup Control Register                  */
  __IO uint32_t WAKUPPOLR;       /*!< 0x054         EXTI Interrupt Wakeup Polarity Register                 */
  __IO uint32_t WAKUPFLG;        /*!< 0x058         EXTI Interrupt Wakeup Flag Register                     */
} HT_EXTI_TypeDef;


/**
 * @brief Inter-integrated Circuit Interface
 */
typedef struct
{
                                 /* I2C2: 0x40008000                                                        */
                                 /* I2C0: 0x40048000                                                        */
                                 /* I2C1: 0x40049000                                                        */
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t IER;             /*!< 0x004         Interrupt Enable Register                               */
  __IO uint32_t ADDR;            /*!< 0x008         Address Register                                        */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t SHPGR;           /*!< 0x010         SCL High Period Generation Register                     */
  __IO uint32_t SLPGR;           /*!< 0x014         SCL Low Period Generation Register                      */
  __IO uint32_t DR;              /*!< 0x018         Data Register                                           */
  __IO uint32_t TAR;             /*!< 0x01C         Target Register                                         */
  __IO uint32_t ADDMR;           /*!< 0x020         Address Mask Register                                   */
  __IO uint32_t ADDSR;           /*!< 0x024         Address Snoop Register                                  */
  __IO uint32_t TOUT;            /*!< 0x028         Timeout Register                                        */
} HT_I2C_TypeDef;


/**
 * @brief WATCHDOG
 */
typedef struct
{
                                 /* WDT: 0x40068000                                                         */
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t MR0;             /*!< 0x004         Mode 0 Register                                         */
  __IO uint32_t MR1;             /*!< 0x008         Mode 1 Register                                         */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t PR;              /*!< 0x010         Write Protect Register                                  */
       uint32_t RESERVED0[1];    /*!< 0x014         Reserved                                                */
  __IO uint32_t CSR;             /*!< 0x018         Clock Selection Register                                */
} HT_WDT_TypeDef;


/**
 * @brief Real-Time Clock
 */
typedef struct
{
                                 /* RTC: 0x4006A000                                                         */
  __IO uint32_t CNT;             /*!< 0x000         RTC Counter Register                                    */
  __IO uint32_t CMP;             /*!< 0x004         RTC Compare Register                                    */
  __IO uint32_t CR;              /*!< 0x008         RTC Control Register                                    */
  __IO uint32_t SR;              /*!< 0x00C         RTC Status Register                                     */
  __IO uint32_t IWEN;            /*!< 0x010         RTC Interrupt/Wake-up Enable Register                   */
} HT_RTC_TypeDef;


/**
 * @brief Power Control Unit
 */
typedef struct
{
                                 /* PWRCU: 0x4006A100                                                       */
  __IO uint32_t SR;              /*!< 0x000         Status Register                                         */
  __IO uint32_t CR;              /*!< 0x004         Control Register                                        */
  __IO uint32_t TEST;            /*!< 0x008         Test Register                                           */
  __IO uint32_t HSIRCR;          /*!< 0x00C         HSI Ready Counter Control Register                      */
  __IO uint32_t LVDCSR;          /*!< 0x010         Low Voltage/Brown Out Detect Control and Status Register*/
       uint32_t RESERVE0[59];    /*!< 0x014 - 0x0FC Reserved                                                */
  __IO uint32_t BAKREG[10];      /*!< 0x100 - 0x124 VDD Power Domain Backup Register 0-9                    */
} HT_PWRCU_TypeDef;


/**
 * @brief Flash Memory Controller
 */
typedef struct
{
                                 /* FLASH: 0x40080000                                                       */
  __IO uint32_t TADR;            /*!< 0x000         Flash Target Address Register                           */
  __IO uint32_t WRDR;            /*!< 0x004         Flash Write Data Register                               */
       uint32_t RESERVED0[1];    /*!< 0x008         Reserved                                                */
  __IO uint32_t OCMR;            /*!< 0x00C         Flash Operation Command Register                        */
  __IO uint32_t OPCR;            /*!< 0x010         Flash Operation Control Register                        */
  __IO uint32_t OIER;            /*!< 0x014         Flash Operation Interrupt Enable Register               */
  __IO uint32_t OISR;            /*!< 0x018         Flash Operation Interrupt and Status Register           */
       uint32_t RESERVED1[1];    /*!< 0x01C         Reserved                                                */
  __IO uint32_t PPSR[4];         /*!< 0x020 ~ 0x02C Flash Page Erase/Program Protection Status Register     */
  __IO uint32_t CPSR;            /*!< 0x030         Flash Security Protection Status Register               */
       uint32_t RESERVED2[51];   /*!< 0x034 ~ 0x0FC Reserved                                                */
  __IO uint32_t VMCR;            /*!< 0x100         Flash Vector Mapping Control Register                   */
       uint32_t RESERVED3[31];   /*!< 0x104 ~ 0x17C Reserved                                                */
  __IO uint32_t MDID;            /*!< 0x180         Manufacturer and Device ID Register                     */
  __IO uint32_t PNSR;            /*!< 0x184         Flash Page Number Status Register                       */
  __IO uint32_t PSSR;            /*!< 0x188         Flash Page Size Status Register                         */
  __IO uint32_t DID;             /*!< 0x18C         Device ID Register                                      */
       uint32_t RESERVED4[28];   /*!< 0x190 ~ 0x1FC Reserved                                                */
  __IO uint32_t WSCR;            /*!< 0x200         Flash Wait State Control Register                       */
       uint32_t RESERVED5[3];    /*!< 0x204 ~ 0x20C Reserved                                                */
  __IO uint32_t CCR;             /*!< 0x210         Flash Cache Controller Configuration and Contorl Register */
  __IO uint32_t SR;              /*!< 0x214         Flash Cache Controller Statuse Register                 */
  __IO uint32_t IRQMASK;         /*!< 0x218         Flash Interrupt Request Mask Register                   */
  __IO uint32_t IRQSTAT;         /*!< 0x21C         Flash Interrupt Request Statuse Register                */
       uint32_t RESERVED6[1];    /*!< 0x220         Reserved                                                */
  __IO uint32_t CSHR;            /*!< 0x224         Flash Cache Statistic Hit Register                      */
  __IO uint32_t CSMR;            /*!< 0x228         Flash Cache Statistic Miss Register                     */
       uint32_t RESERVED7[57];   /*!< 0x22C ~ 0x30C Reserved                                                */
  __IO uint32_t UID[3];          /*!< 0x310 ~ 0x318 Unique ID Register 0-2                                  */
  __IO uint32_t CID;             /*!< 0x31C         Custom ID Register                                      */
  __IO uint32_t SPR;             /*!< 0x320         Special Purpose Register                                */
} HT_FLASH_TypeDef;


/**
 * @brief Clock Control Unit
 */
typedef struct
{
                                 /* CKCU: 0x40088000                                                        */
  __IO uint32_t GCFGR;           /*!< 0x000         Global Clock Configuration Register                     */
  __IO uint32_t GCCR;            /*!< 0x004         Global Clock Control Register                           */
  __IO uint32_t GCSR;            /*!< 0x008         Global Clock Status Register                            */
  __IO uint32_t GCIR;            /*!< 0x00C         Global Clock Interrupt Register                         */
       uint32_t RESERVED0[2];    /*!< 0x010 ~ 0x14  Reserved                                                */
  __IO uint32_t PLLCFGR;         /*!< 0x018         PLL Configuration Register                              */
  __IO uint32_t PLLCR;           /*!< 0x01C         PLL Control Register                                    */
  __IO uint32_t AHBCFGR;         /*!< 0x020         AHB Configuration Register                              */
  __IO uint32_t AHBCCR;          /*!< 0x024         AHB Clock Control Register                              */
  __IO uint32_t APBCFGR;         /*!< 0x028         APB Configuration Register                              */
  __IO uint32_t APBCCR0;         /*!< 0x02C         APB Clock Control Register 0                            */
  __IO uint32_t APBCCR1;         /*!< 0x030         APB Clock Control Register 1                            */
  __IO uint32_t CKST;            /*!< 0x034         Clock source status Register                            */
  __IO uint32_t APBPCSR0;        /*!< 0x038         APB Peripheral Clock Selection Register 0               */
  __IO uint32_t APBPCSR1;        /*!< 0x03C         APB Peripheral Clock Selection Register 1               */
  __IO uint32_t HSICR;           /*!< 0x040         HSI Control Register                                    */
  __IO uint32_t HSIATCR;         /*!< 0x044         HSI Auto Trimming Counter Register                      */
       uint32_t RESERVED1[1];    /*!< 0x048         Reserved                                                */
  __IO uint32_t HSIRDYCR;        /*!< 0x04C         HSI Ready Counter Register                              */
  __IO uint32_t LSITCR;          /*!< 0x050         LSI Trim Control Register                               */
       uint32_t RESERVED2[172];  /*!< 0x054 ~ 0x300 Reserved                                                */
  __IO uint32_t MCUDBGCR;        /*!< 0x304         MCU Debug Control Register                              */
} HT_CKCU_TypeDef;


/**
 * @brief Reset Control Unit
 */
typedef struct
{
                                 /* RSTCU: 0x40088100                                                       */
  __IO uint32_t GRSR;            /*!< 0x000         Global Reset Status Register                            */
  __IO uint32_t AHBPRST;         /*!< 0x004         AHB Peripheral Reset Register                           */
  __IO uint32_t APBPRST0;        /*!< 0x008         APB Peripheral Reset Register 0                         */
  __IO uint32_t APBPRST1;        /*!< 0x00C         APB Peripheral Reset Register 1                         */
} HT_RSTCU_TypeDef;


/**
 * @brief Basic Function Timer
 */
typedef struct
{
                                 /* BFTM0: 0x40076000                                                       */
                                 /* BFTM1: 0x40077000                                                       */
  __IO uint32_t CR;              /*!< 0x000          Control Register                                       */
  __IO uint32_t SR;              /*!< 0x004          Status Register                                        */
  __IO uint32_t CNTR;            /*!< 0x008          Counter Value Register                                 */
  __IO uint32_t CMP;             /*!< 0x00C          Compare Value Register                                 */
} HT_BFTM_TypeDef;


/**
 * @brief Timer
 */
typedef struct
{
  __IO uint32_t CNTCFR;          /*!< 0x000          Counter Configuration Register                         */
  __IO uint32_t MDCFR;           /*!< 0x004          Mode Configuration Register                            */
  __IO uint32_t TRCFR;           /*!< 0x008          Trigger Configuration Register                         */
       uint32_t RESERVED0[1];    /*!< 0x00C          Reserved                                               */
  __IO uint32_t CTR;             /*!< 0x010          Control Register                                       */
       uint32_t RESERVED1[3];    /*!< 0x014 - 0x01C  Reserved                                               */
  __IO uint32_t CH0ICFR;         /*!< 0x020          Channel-0 Input Configuration Register                 */
  __IO uint32_t CH1ICFR;         /*!< 0x024          Channel-1 Input Configuration Register                 */
  __IO uint32_t CH2ICFR;         /*!< 0x028          Channel-2 Input Configuration Register                 */
  __IO uint32_t CH3ICFR;         /*!< 0x02C          Channel-3 Input Configuration Register                 */
       uint32_t RESERVED2[4];    /*!< 0x030 - 0x03C  Reserved                                               */
  __IO uint32_t CH0OCFR;         /*!< 0x040          Channel-0 Output Configuration Register                */
  __IO uint32_t CH1OCFR;         /*!< 0x044          Channel-1 Output Configuration Register                */
  __IO uint32_t CH2OCFR;         /*!< 0x048          Channel-2 Output Configuration Register                */
  __IO uint32_t CH3OCFR;         /*!< 0x04C          Channel-3 Output Configuration Register                */
  __IO uint32_t CHCTR;           /*!< 0x050          Channel Control Register                               */
  __IO uint32_t CHPOLR;          /*!< 0x054          Channel Polarity Configuration Register                */
       uint32_t RESERVED3[5];    /*!< 0x058 - 0x068  Reserved                                               */
  __IO uint32_t CHBRKCFR;        /*!< 0x06C          Channel Break Configuration Register                   */
  __IO uint32_t CHBRKCTR;        /*!< 0x070          Channel Break Control Register                         */
  __IO uint32_t DICTR;           /*!< 0x074          DMA / Interrupt Control Register                       */
  __IO uint32_t EVGR;            /*!< 0x078          Event Generator Register                               */
  __IO uint32_t INTSR;           /*!< 0x07C          Interrupt Status Register                              */
  __IO uint32_t CNTR;            /*!< 0x080          Counter Register                                       */
  __IO uint32_t PSCR;            /*!< 0x084          Prescaler Register                                     */
  __IO uint32_t CRR;             /*!< 0x088          Counter Reload Register                                */
  __IO uint32_t REPR;            /*!< 0x08C          Repetition Register                                    */
  __IO uint32_t CH0CCR;          /*!< 0x090          Channel 0 Capture/Compare Register                     */
  __IO uint32_t CH1CCR;          /*!< 0x094          Channel 1 Capture/Compare Register                     */
  __IO uint32_t CH2CCR;          /*!< 0x098          Channel 2 Capture/Compare Register                     */
  __IO uint32_t CH3CCR;          /*!< 0x09C          Channel 3 Capture/Compare Register                     */
  __IO uint32_t CH0ACR;          /*!< 0x0A0          Channel 0 Asymmetric Compare Register                  */
  __IO uint32_t CH1ACR;          /*!< 0x0A4          Channel 1 Asymmetric Compare Register                  */
  __IO uint32_t CH2ACR;          /*!< 0x0A8          Channel 2 Asymmetric Compare Register                  */
  __IO uint32_t CH3ACR;          /*!< 0x0AC          Channel 3 Asymmetric Compare Register                  */
} HT_TM_TypeDef;


/**
 * @brief Peripheral Direct Memory Access Channel
 */
typedef struct
{
  __IO uint32_t CR;              /*!< 0x000    PDMA Channel Control Register                                */
  __IO uint32_t SADR;            /*!< 0x004    PDMA Channel Source Address Register                         */
  __IO uint32_t DADR;            /*!< 0x008    PDMA Channel Destination Address Register                    */
       uint32_t RESERVED0[1];    /*!< 0x00C    Reserved                                                     */
  __IO uint32_t TSR;             /*!< 0x010    PDMA Channel Transfer Size Register                          */
  __IO uint32_t CTSR;            /*!< 0x014    PDMA Channel Current Transfer Size Register                  */
} HT_PDMACH_TypeDef;


/**
 * @brief Peripheral Direct Memory Access Global
 */
typedef struct
{
                                  /* PDMA: 0x40090000                                                       */
  HT_PDMACH_TypeDef PDMACH0;      /*!< 0x000          PDMA channel  0 registers                             */
  HT_PDMACH_TypeDef PDMACH1;      /*!< 0x018          PDMA channel  1 registers                             */
  HT_PDMACH_TypeDef PDMACH2;      /*!< 0x030          PDMA channel  2 registers                             */
  HT_PDMACH_TypeDef PDMACH3;      /*!< 0x048          PDMA channel  3 registers                             */
  HT_PDMACH_TypeDef PDMACH4;      /*!< 0x060          PDMA channel  4 registers                             */
  HT_PDMACH_TypeDef PDMACH5;      /*!< 0x078          PDMA channel  5 registers                             */
       uint32_t     RESERVED0[36];/*!< 0x090 - 0x11C  Reserved                                              */
  __IO uint32_t     ISR;          /*!< 0x120          PDMA Interrupt Status Register                        */
       uint32_t     RESERVED1[1]; /*!< 0x124          Reserved                                              */
  __IO uint32_t     ISCR;         /*!< 0x128          PDMA Interrupt Status Clear Register                  */
       uint32_t     RESERVED2[1]; /*!< 0x12C          Reserved                                              */
  __IO uint32_t     IER;          /*!< 0x130          PDMA Interrupt Enable Register                        */
} HT_PDMA_TypeDef;

/**
 * @brief Cyclic Redundancy Check
 */
typedef struct
{
                                 /* CRC: 0x4008A000                                                         */
  __IO uint32_t CR;              /*!< 0x000 CRC Control Register                                            */
  __IO uint32_t SDR;             /*!< 0x004 CRC Seed Register                                               */
  __IO uint32_t CSR;             /*!< 0x008 CRC Checksum Register                                           */
  __IO uint32_t DR;              /*!< 0x00C CRC Data Register                                               */
} HT_CRC_TypeDef;


/**
 * @brief Divider
 */
typedef struct
{
                                 /* DIV: 0x400CA000                                                         */
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t DDR;             /*!< 0x004         Dividend register                                       */
  __IO uint32_t DSR;             /*!< 0x008         Divisor Register                                        */
  __IO uint32_t QTR;             /*!< 0x00C         Quotient Register                                       */
  __IO uint32_t RMR;             /*!< 0x010         Remainder Register                                      */
} HT_DIV_TypeDef;


/** @addtogroup Peripheral_Memory_Map
  * @{
  */

#define HT_SRAM_BASE             (0x20000000UL)

#define HT_PERIPH_BASE           (0x40000000UL)

#define HT_APBPERIPH_BASE        (HT_PERIPH_BASE)                 /* 0x40000000                             */
#define HT_AHBPERIPH_BASE        (HT_PERIPH_BASE + 0x80000)       /* 0x40080000                             */

/* APB                                                                                                      */
#define HT_USART0_BASE           (HT_APBPERIPH_BASE + 0x0000)     /* 0x40000000                             */
#define HT_UART0_BASE            (HT_APBPERIPH_BASE + 0x1000)     /* 0x40001000                             */
#define HT_UART2_BASE            (HT_APBPERIPH_BASE + 0x2000)     /* 0x40002000                             */
#define HT_SPI0_BASE             (HT_APBPERIPH_BASE + 0x4000)     /* 0x40004000                             */
#define HT_I2C2_BASE             (HT_APBPERIPH_BASE + 0x8000)     /* 0x40008000                             */
#define HT_ADC0_BASE             (HT_APBPERIPH_BASE + 0x10000)    /* 0x40010000                             */
#define HT_AFIO_BASE             (HT_APBPERIPH_BASE + 0x22000)    /* 0x40022000                             */
#define HT_EXTI_BASE             (HT_APBPERIPH_BASE + 0x24000)    /* 0x40024000                             */
#define HT_MCTM0_BASE            (HT_APBPERIPH_BASE + 0x2C000)    /* 0x4002C000                             */
#define HT_SCTM0_BASE            (HT_APBPERIPH_BASE + 0x34000)    /* 0x40034000                             */
#define HT_SCTM2_BASE            (HT_APBPERIPH_BASE + 0x35000)    /* 0x40035000                             */
#define HT_USART1_BASE           (HT_APBPERIPH_BASE + 0x40000)    /* 0x40040000                             */
#define HT_UART1_BASE            (HT_APBPERIPH_BASE + 0x41000)    /* 0x40041000                             */
#define HT_UART3_BASE            (HT_APBPERIPH_BASE + 0x42000)    /* 0x40042000                             */
#define HT_SPI1_BASE             (HT_APBPERIPH_BASE + 0x44000)    /* 0x40044000                             */
#define HT_I2C0_BASE             (HT_APBPERIPH_BASE + 0x48000)    /* 0x40048000                             */
#define HT_I2C1_BASE             (HT_APBPERIPH_BASE + 0x49000)    /* 0x40049000                             */
#define HT_ADC1_BASE             (HT_APBPERIPH_BASE + 0x50000)    /* 0x40050000                             */
#define HT_WDT_BASE              (HT_APBPERIPH_BASE + 0x68000)    /* 0x40068000                             */
#define HT_RTC_BASE              (HT_APBPERIPH_BASE + 0x6A000)    /* 0x4006A000                             */
#define HT_PWRCU_BASE            (HT_APBPERIPH_BASE + 0x6A100)    /* 0x4006A100                             */
#define HT_GPTM0_BASE            (HT_APBPERIPH_BASE + 0x6E000)    /* 0x4006E000                             */
#define HT_GPTM1_BASE            (HT_APBPERIPH_BASE + 0x6F000)    /* 0x4006F000                             */
#define HT_SCTM1_BASE            (HT_APBPERIPH_BASE + 0x74000)    /* 0x40074000                             */
#define HT_SCTM3_BASE            (HT_APBPERIPH_BASE + 0x75000)    /* 0x40075000                             */
#define HT_BFTM0_BASE            (HT_APBPERIPH_BASE + 0x76000)    /* 0x40076000                             */
#define HT_BFTM1_BASE            (HT_APBPERIPH_BASE + 0x77000)    /* 0x40077000                             */

/* AHB                                                                                                      */
#define HT_FLASH_BASE            (HT_AHBPERIPH_BASE + 0x0000)     /* 0x40080000                             */
#define HT_CKCU_BASE             (HT_AHBPERIPH_BASE + 0x8000)     /* 0x40088000                             */
#define HT_RSTCU_BASE            (HT_AHBPERIPH_BASE + 0x8100)     /* 0x40088100                             */
#define HT_CRC_BASE              (HT_AHBPERIPH_BASE + 0xA000)     /* 0x4008A000                             */
#define HT_PDMA_BASE             (HT_AHBPERIPH_BASE + 0x10000)    /* 0x40090000                             */
#define HT_GPIOA_BASE            (HT_AHBPERIPH_BASE + 0x30000)    /* 0x400B0000                             */
#define HT_GPIOB_BASE            (HT_AHBPERIPH_BASE + 0x32000)    /* 0x400B2000                             */
#define HT_GPIOC_BASE            (HT_AHBPERIPH_BASE + 0x34000)    /* 0x400B4000                             */
#define HT_GPIOD_BASE            (HT_AHBPERIPH_BASE + 0x36000)    /* 0x400B6000                             */
#define HT_GPIOE_BASE            (HT_AHBPERIPH_BASE + 0x38000)    /* 0x400B8000                             */
#define HT_GPIOF_BASE            (HT_AHBPERIPH_BASE + 0x3A000)    /* 0x400BA000                             */
#define HT_DIV_BASE              (HT_AHBPERIPH_BASE + 0x4A000)    /* 0x400CA000                             */

/**
  * @}
  */

/* Peripheral declaration                                                                                   */
#define HT_FLASH                 ((HT_FLASH_TypeDef *) HT_FLASH_BASE)
#define HT_CKCU                  ((HT_CKCU_TypeDef *) HT_CKCU_BASE)
#define HT_PWRCU                 ((HT_PWRCU_TypeDef *) HT_PWRCU_BASE)
#define HT_RSTCU                 ((HT_RSTCU_TypeDef *) HT_RSTCU_BASE)
#define HT_AFIO                  ((HT_AFIO_TypeDef *) HT_AFIO_BASE)
#define HT_EXTI                  ((HT_EXTI_TypeDef *) HT_EXTI_BASE)
#define HT_GPIOA                 ((HT_GPIO_TypeDef *) HT_GPIOA_BASE)
#define HT_GPIOB                 ((HT_GPIO_TypeDef *) HT_GPIOB_BASE)
#define HT_BFTM0                 ((HT_BFTM_TypeDef *) HT_BFTM0_BASE)
#define HT_WDT                   ((HT_WDT_TypeDef *) HT_WDT_BASE)
#define HT_UART0                 ((HT_USART_TypeDef *) HT_UART0_BASE)
#define HT_SPI0                  ((HT_SPI_TypeDef *) HT_SPI0_BASE)
#define HT_I2C0                  ((HT_I2C_TypeDef *) HT_I2C0_BASE)
#define HT_ADC0                  ((HT_ADC_TypeDef *) HT_ADC0_BASE)
#define HT_USART0                ((HT_USART_TypeDef *) HT_USART0_BASE)
#define HT_SCTM0                 ((HT_TM_TypeDef *) HT_SCTM0_BASE)
#define HT_SCTM1                 ((HT_TM_TypeDef *) HT_SCTM1_BASE)
#define HT_GPTM0                 ((HT_TM_TypeDef *) HT_GPTM0_BASE)

#if defined(USE_HT32L52231_41)
#define HT_CRC                   ((HT_CRC_TypeDef *) HT_CRC_BASE)
#define HT_DIV                   ((HT_DIV_TypeDef *) HT_DIV_BASE)
#define HT_PDMA                  ((HT_PDMA_TypeDef *) HT_PDMA_BASE)
#define HT_GPIOC                 ((HT_GPIO_TypeDef *) HT_GPIOC_BASE)
#define HT_UART1                 ((HT_USART_TypeDef *) HT_UART1_BASE)
#define HT_SPI1                  ((HT_SPI_TypeDef *) HT_SPI1_BASE)
#define HT_I2C1                  ((HT_I2C_TypeDef *) HT_I2C1_BASE)
#define HT_RTC                   ((HT_RTC_TypeDef *) HT_RTC_BASE)
#define HT_MCTM0                 ((HT_TM_TypeDef *) HT_MCTM0_BASE)
#define HT_BFTM1                 ((HT_BFTM_TypeDef *) HT_BFTM1_BASE)
#endif

#if defined USE_HT32_DRIVER
  #include "ht32l5xxxx_lib.h"
#endif

/**
 * @brief Adjust the value of High Speed External oscillator (HSE)
          Tip: To avoid from modifying every time for different HSE, please define
          the "HSE_VALUE=n000000" ("n" represents n MHz) in your own toolchain compiler preprocessor,
          or edit the "HSE_VALUE" in the "ht32l5xxxx_conf.h" file.
  */
#if !defined  HSE_VALUE
/* Available HSE_VALUE: 4 MHz ~ 16 MHz                                                                  */
#define HSE_VALUE       8000000UL  /*!< Value of the External oscillator in Hz                          */
#endif

/**
 * @brief Define for backward compatibility
 */
#define HT_ADC                   HT_ADC0
#define ADC                      ADC0
#define ADC_IRQn                 ADC0_IRQn

#define HT_DAC                   HT_DAC0
#define AFIO_FUN_DAC             AFIO_FUN_DAC0
#define CKCU_PCLK_DAC            CKCU_PCLK_DAC0

#define AFIO_ESS_Enum            u32

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Holtek Semiconductor Inc. *****END OF FILE***                            */
