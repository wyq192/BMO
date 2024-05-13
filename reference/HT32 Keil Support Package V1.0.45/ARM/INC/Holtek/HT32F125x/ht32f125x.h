/***************************************************************************//**
 * @file    ht32f125x.h
 * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File
 * @version V1.00
 * @date    04/11/2011
 *
 * @note
 * Copyright (C) 2011 Holtek Semiconductor Inc. All rights reserved.
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup HT32F125x
  * @{
  */

#ifndef __HT32F125x_H__
#define __HT32F125x_H__

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

#define HSI_VALUE         8000000UL  /*!< Value of the Internal oscillator in Hz                            */

/**
 * @brief Adjust the value of External High Speed oscillator (HSE)

   Tip: To avoid from modifying every time for different HSE, please define
        the HSE value in your own toolchain compiler preprocessor.
  */
#if !defined  HSE_VALUE
  /* Available HSE_VALUE: 4MHz ~ 16MHz                                                                      */
  #define HSE_VALUE       8000000UL   /*!< Value of the External oscillator in Hz                           */
#endif


/**
 * @brief Adjust the External High Speed oscillator (HSE) Startup Timeout value
   */
#define HSE_READY_TIME    ((uint16_t)0xFFFF)    /*!< Time out for HSE start up                              */




/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
#define __MPU_PRESENT             0    /*!< MPU present or not                                              */
#define __NVIC_PRIO_BITS          4    /*!< Number of Bits used for Priority Levels                         */
#define __Vendor_SysTickConfig    0    /*!< Set to 1 if different SysTick Config is used                    */

/**
  * @}
  */


/** @addtogroup Configuration_for_Interrupt_Number
  * @{
  */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ******************************                              */
  NonMaskableInt_IRQn     = -14,    /*!< 2 Non Maskable Interrupt                                           */
  HardFault_IRQn          = -13,    /*!< 3 Cortex-M3 Hard Fault Interrupt                                   */
  MemoryManagement_IRQn   = -12,    /*!< 4 Cortex-M3 Memory Management Int                                  */
  BusFault_IRQn           = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                                    */
  UsageFault_IRQn         = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                                  */
  SVCall_IRQn             = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                                     */
  DebugMonitor_IRQn       = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                               */
  PendSV_IRQn             = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                                     */
  SysTick_IRQn            = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                                 */

/******  HT32 Specific Interrupt Numbers ************************************                               */
  CKRDY_IRQn              = 0,      /*!< Clock ready interrupt                                              */
  LVD_IRQn                = 1,      /*!< Low voltage detection interrupt                                    */
  BOD_IRQn                = 2,      /*!< Brown-Out detection interrupt                                      */
  WDT_IRQn                = 3,      /*!< WDT global interrupt                                               */
  RTC_IRQn                = 4,      /*!< RTC Wake-up Interrupt                                              */
  FLASH_IRQn              = 5,      /*!< FLASH global Interrupt                                             */
  EVWUP_IRQn              = 6,      /*!< Event Wake-up Interrupt                                            */
  LPWUP_IRQn              = 7,      /*!< WAKEUP pin Interrupt                                               */
  EXTI0_IRQn              = 8,      /*!< EXTI0 Line detection Interrupt                                     */
  EXTI1_IRQn              = 9,      /*!< EXTI1 Line detection Interrupt                                     */
  EXTI2_IRQn              = 10,     /*!< EXTI2 Line detection Interrupt                                     */
  EXTI3_IRQn              = 11,     /*!< EXTI3 Line detection Interrupt                                     */
  EXTI4_IRQn              = 12,     /*!< EXTI4 Line detection Interrupt                                     */
  EXTI5_IRQn              = 13,     /*!< EXTI5 Line detection Interrupt                                     */
  EXTI6_IRQn              = 14,     /*!< EXTI6 Line detection Interrupt                                     */
  EXTI7_IRQn              = 15,     /*!< EXTI7 Line detection Interrupt                                     */
  EXTI8_IRQn              = 16,     /*!< EXTI8 Line detection Interrupt                                     */
  EXTI9_IRQn              = 17,     /*!< EXTI9 Line detection Interrupt                                     */
  EXTI10_IRQn             = 18,     /*!< EXTI10 Line detection Interrupt                                    */
  EXTI11_IRQn             = 19,     /*!< EXTI11 Line detection Interrupt                                    */
  EXTI12_IRQn             = 20,     /*!< EXTI12 Line detection Interrupt                                    */
  EXTI13_IRQn             = 21,     /*!< EXTI13 Line detection Interrupt                                    */
  EXTI14_IRQn             = 22,     /*!< EXTI14 Line detection Interrupt                                    */
  EXTI15_IRQn             = 23,     /*!< EXTI15 Line detection Interrupt                                    */
  COMP_IRQn               = 24,     /*!< Comparator global Interrupt                                        */
  ADC_IRQn                = 25,     /*!< ADC Interrupt                                                      */
  GPTM0_IRQn              = 35,     /*!< General-Purpose Timer0 Interrupt                                   */
  GPTM1_IRQn              = 36,     /*!< General-Purpose Timer1 Interrupt                                   */
  I2C_IRQn                = 43,     /*!< I2C global Interrupt                                               */
  SPI_IRQn                = 45,     /*!< SPI global Interrupt                                               */
  USART_IRQn              = 47      /*!< USART global Interrupt                                             */
} IRQn_Type;


/**
  * @}
  */

#include "core_cm3.h"                  /* Cortex-M3 processor and core peripherals                          */
#include "system_ht32f125x.h"          /* HT32 system                                                       */


/** @addtogroup Exported_Types
  * @{
  */

typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef const int32_t sc32;           /*!< Read Only                                                        */
typedef const int16_t sc16;           /*!< Read Only                                                        */
typedef const int8_t  sc8;            /*!< Read Only                                                        */

typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t  vs8;

typedef __I int32_t vsc32;            /*!< Read Only                                                        */
typedef __I int16_t vsc16;            /*!< Read Only                                                        */
typedef __I int8_t  vsc8;             /*!< Read Only                                                        */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;    /*!< Read Only                                                        */
typedef unsigned short const uc16;    /*!< Read Only                                                        */
typedef unsigned char  const uc8;     /*!< Read Only                                                        */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;

typedef __I uint32_t vuc32;           /*!< Read Only                                                        */
typedef __I uint16_t vuc16;           /*!< Read Only                                                        */
typedef __I uint8_t  vuc8;            /*!< Read Only                                                        */


typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
#define IS_CONTROL_STATUS(STATUS) ((STATUS == DISABLE) || (STATUS == ENABLE))

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

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

#if defined ( __CC_ARM   )
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
#define wb(addr, value)     (*((u8  volatile *) (addr)) = value)
#define rb(addr)            (*((u8  volatile *) (addr)))
#define whw(addr, value)    (*((u16 volatile *) (addr)) = value)
#define rhw(addr)           (*((u16 volatile *) (addr)))
#define ww(addr, value)     (*((u32 volatile *) (addr)) = value)
#define rw(addr)            (*((u32 volatile *) (addr)))


#define ResetBit_BB(Addr, BitNumber) (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)) = 0)
#define SetBit_BB(Addr, BitNumber)   (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)) = 1)
#define GetBit_BB(Addr, BitNumber)   (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)))
#define BitBand(Addr, BitNumber)     (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                     ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)))


#ifndef EXT
  #define EXT extern
#endif

/** @addtogroup Peripheral_Registers_Structures
  * @{
  */


/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */
typedef struct
{
  union {
  __IO uint32_t RBR;             /*!< 0x000         Receive Buffer Register                                 */
  __IO uint32_t TBR;             /*!< 0x000         Transmit Holding Register                               */
  };
  __IO uint32_t IER;             /*!< 0x004         Interrupt Enable Register                               */
  __IO uint32_t IIR;             /*!< 0x008         Interrupt Identification Register/FIFO Control Register */
  __IO uint32_t FCR;             /*!< 0x00C         FIFO Control Register                                   */
  __IO uint32_t LCR;             /*!< 0x010         Line Control Register                                   */
  __IO uint32_t MCR;             /*!< 0x014         Modem Control Register                                  */
  __IO uint32_t LSR;             /*!< 0x018         Line Status Register                                    */
  __IO uint32_t MSR;             /*!< 0x01C         Modem Status Register                                   */
  __IO uint32_t TPR;             /*!< 0x020         Timing Parameter Register                               */
  __IO uint32_t MDR;             /*!< 0x024         Mode Register                                           */
  __IO uint32_t ICR;             /*!< 0x028         IrDA Register                                           */
  __IO uint32_t RCR;             /*!< 0x02C         RS485 Control Register                                  */
  __IO uint32_t SCR;             /*!< 0x030         Synchronous Control Register                            */
  __IO uint32_t DTR;             /*!< 0x034         Debug/Test Register                                     */
  __IO uint32_t DLR;             /*!< 0x038         Divisor Latch Register                                  */
} HT_USART_TypeDef;


/**
 * @brief SPI
 */
typedef struct
{
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
       uint32_t RESERVE0[1];     /*!< 0x000         Reserved                                                */
  __IO uint32_t RST;             /*!< 0x004         ADC Reset Register                                      */
  __IO uint32_t CONV;            /*!< 0x008         ADC (Regular) Conversion Mode Register                  */
       uint32_t RESERVE1[1];     /*!< 0x00C         Reserved                                                */
  __IO uint32_t LST[2];          /*!< 0x010 - 0x014 ADC (Regular) Conversion List Register 0-1              */
       uint32_t RESERVE2[22];    /*!< 0x018 - 0x06C Reserved                                                */
  __IO uint32_t STR[8];          /*!< 0x070 - 0x08C ADC Input Sampling Time Register 0-7                    */
       uint32_t RESERVE3[8];     /*!< 0x090 - 0x0AC Reserved                                                */
  __IO uint32_t DR[8];           /*!< 0x0B0 - 0x0CC ADC (Regular) Conversion Data Register 0-7              */
       uint32_t RESERVE4[12];    /*!< 0x0D0 - 0x0FC Reserved                                                */
  __IO uint32_t TCR;             /*!< 0x100         ADC (Regular) Trigger Control Register                  */
  __IO uint32_t TSR;             /*!< 0x104         ADC (Regular) Trigger Source Register                   */
       uint32_t RESERVE5[6];     /*!< 0x108 - 0x11C Reserved                                                */
  __IO uint32_t WCR;             /*!< 0x120         ADC Watchdog Control Register                           */
  __IO uint32_t LTR;             /*!< 0x124         ADC Lower Threshold Register                            */
  __IO uint32_t UTR;             /*!< 0x128         ADC Upper Threshold Register                            */
       uint32_t RESERVE6[1];     /*!< 0x12C         Reserved                                                */
  __IO uint32_t IM;              /*!< 0x130         ADC Interrupt Mask Enable Register                      */
  __IO uint32_t IRAW;            /*!< 0x134         ADC Interrupt Raw Status Register                       */
  __IO uint32_t IMASK;           /*!< 0x138         ADC Interrupt Masked Status Register                    */
  __IO uint32_t ICLR;            /*!< 0x13C         ADC Interrupt Clear Register                            */
} HT_ADC_TypeDef;


/**
 * @brief Op Amp/Comparator
 */
typedef struct
{
  __IO uint32_t OPACR;          /*!< 0x000         Operational Amplifier control register                   */
  __IO uint32_t OFVCR;          /*!< 0x004         Comparator input offset voltage cancellation register    */
  __IO uint32_t CMPIER;         /*!< 0x008         Comparator interrupt enable register                     */
  __IO uint32_t CMPRSR;         /*!< 0x00C         Comparator raw status register                           */
  __IO uint32_t CMPISR;         /*!< 0x010         Comparator interrupt status register                     */
  __IO uint32_t CMPICLR;        /*!< 0x014         Comparator interrupt clear register                      */
} HT_CMP_OP_TypeDef;


/**
 * @brief General Purpose I/O
 */
typedef struct
{
  __IO uint32_t DIRCR;          /*!< 0x000         Data Direction Control Register                          */
  __IO uint32_t INER;           /*!< 0x004         Input function enable register                           */
  __IO uint32_t PUR;            /*!< 0x008         Pull-Up Selection Register                               */
  __IO uint32_t PDR;            /*!< 0x00C         Pull-Down Selection Register                             */
  __IO uint32_t ODR;            /*!< 0x010         Open Drain Selection Register                            */
  __IO uint32_t DRVR;           /*!< 0x014         Drive Current Selection Register                         */
  __IO uint32_t LOCKR;          /*!< 0x018         Lock Register                                            */
  __IO uint32_t DINR;           /*!< 0x01c         Data Input Register                                      */
  __IO uint32_t DOUTR;          /*!< 0x020         Data Output Register                                     */
  __IO uint32_t SRR;            /*!< 0x024         Output Set and Reset Control Register                    */
  __IO uint32_t RR;             /*!< 0x028         Output Reset Control Register                            */
} HT_GPIO_TypeDef;


/**
 * @brief AFIO
 */
typedef struct
{
  __IO uint32_t ESSR[2];        /*!< 0x000         EXTI Source Selection Register 0 ~ 1                     */
  __IO uint32_t GPACFGR;        /*!< 0x008         GPIO A Configuration Register                            */
  __IO uint32_t GPBCFGR;        /*!< 0x00C         GPIO B Configuration Register                            */
} HT_AFIO_TypeDef;


/**
 * @brief External Interrupt/Event Controller
 */
typedef struct
{
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
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t IER;             /*!< 0x004         Interrupt Enable Register                               */
  __IO uint32_t ADDR;            /*!< 0x008         Address Register                                        */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t SHPGR;           /*!< 0x010         SCL High Period Generation Register                     */
  __IO uint32_t SLPGR;           /*!< 0x014         SCL Low Period Generation Register                      */
  __IO uint32_t DR;              /*!< 0x018         Data Register                                           */
  __IO uint32_t TAR;             /*!< 0x01C         Target Register                                         */
} HT_I2C_TypeDef;


/**
 * @brief WATCHDOG
 */
typedef struct
{
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t MR0;             /*!< 0x004         Mode 0 Register                                         */
  __IO uint32_t MR1;             /*!< 0x008         Mode 1 Register                                         */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t PR;              /*!< 0x010         WDT Write Protect Register                              */
} HT_WDT_TypeDef;


/**
 * @brief Real-Time Clock
 */
typedef struct
{
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
  __IO uint32_t BAKSR;           /*!< 0x000         Backup Domain Status Register                           */
  __IO uint32_t BAKCR;           /*!< 0x004         Backup Domain Control Register                          */
  __IO uint32_t BAKTEST;         /*!< 0x008         Backup Domain Test Register                             */
  __IO uint32_t HSIRCR;          /*!< 0x00C         HSI Ready Counter Control Register                      */
  __IO uint32_t LVDCSR;          /*!< 0x010         Low Voltage/Brown Out Detect Control and Status Register*/
       uint32_t RESERVE0[59];    /*!< 0x014 ~ 0x0FC Reserved                                                */
  __IO uint32_t BAKREG[10];      /*!< 0x100 ~ 0x124 Backup Register 0 ~ 9                                   */
} HT_PWRCU_TypeDef;


/**
 * @brief General-Purpose Timer
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
       uint32_t RESERVED3[7];    /*!< 0x058 - 0x070  Reserved                                               */
  __IO uint32_t ICTR;            /*!< 0x074          Interrupt Control Register                             */
  __IO uint32_t EVGR;            /*!< 0x078          Software Interrupt Control Register                    */
  __IO uint32_t INTSR;           /*!< 0x07C          Interrupt Status Register                              */
  __IO uint32_t CNTR;            /*!< 0x080          Counter Register                                       */
  __IO uint32_t PSCR;            /*!< 0x084          Prescaler Register                                     */
  __IO uint32_t CRR;             /*!< 0x088          Counter Reload Register                                */
       uint32_t RESERVED4[1];    /*!< 0x08C          Reserved                                               */
  __IO uint32_t CH0CCR;          /*!< 0x090          Channel 0 Capture/Compare Register                     */
  __IO uint32_t CH1CCR;          /*!< 0x094          Channel 1 Capture/Compare Register                     */
  __IO uint32_t CH2CCR;          /*!< 0x098          Channel 2 Capture/Compare Register                     */
  __IO uint32_t CH3CCR;          /*!< 0x09C          Channel 3 Capture/Compare Register                     */
} HT_GPTM_TypeDef;


/**
 * @brief Flash Memory Controller
 */
typedef struct
{
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
       uint32_t RESERVED3[63];   /*!< 0x104 ~ 0x1FC Reserved                                                */
  __IO uint32_t CFCR;            /*!< 0x200         Flash Pre-fetch Control Register                        */
       uint32_t RESERVED4[63];   /*!< 0x204 ~ 0x2FC Reserved                                                */
  __IO uint32_t SBVT[4];         /*!< 0x300 ~ 0x30C SRAM Booting Vector (4x32Bit)                           */
} HT_FLASH_TypeDef;


/**
 * @brief Clock Control Unit
 */
typedef struct
{
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
       uint32_t RESERVED1[178];  /*!< 0x038 ~ 0x2FC Reserved                                                */
  __IO uint32_t LPCR;            /*!< 0x300         Low Power Control Register                              */
  __IO uint32_t MCUDBGCR;        /*!< 0x304         MCU Debug Control Register                              */
} HT_CKCU_TypeDef;


/**
 * @brief Reset Control Unit
 */
typedef struct
{
  __IO uint32_t GRSR;            /*!< 0x000         Global Reset Status Register                            */
       uint32_t RESERVED0[1];    /*!< 0x004         Reserved                                                */
  __IO uint32_t APBPRST0;        /*!< 0x008         APB Peripheral Reset Register 0                         */
  __IO uint32_t APBPRST1;        /*!< 0x00C         APB Peripheral Reset Register 1                         */
} HT_RSTCU_TypeDef;


/**
  * @}
  */


/** @addtogroup Peripheral_Memory_Map
  * @{
  */

#define HT_SRAM_BASE             ((u32)0x20000000)
#define HT_SRAM_BB_BASE          ((u32)0x22000000)

#define HT_PERIPH_BASE           ((u32)0x40000000)
#define HT_PERIPH_BB_BASE        ((u32)0x42000000)

#define HT_APB0PERIPH_BASE       (HT_PERIPH_BASE)                 /* 0x40000000                             */
#define HT_APB1PERIPH_BASE       (HT_PERIPH_BASE + 0x40000)       /* 0x40040000                             */
#define HT_AHBPERIPH_BASE        (HT_PERIPH_BASE + 0x80000)       /* 0x40080000                             */

/* APB0                                                                                                     */
#define HT_USART_BASE            (HT_APB0PERIPH_BASE + 0x0000)    /* 0x40000000                             */
#define HT_SPI_BASE              (HT_APB0PERIPH_BASE + 0x4000)    /* 0x40004000                             */
#define HT_ADC_BASE              (HT_APB0PERIPH_BASE + 0x10000)   /* 0x40010000                             */
#define HT_CMP_OP0_BASE          (HT_APB0PERIPH_BASE + 0x18000)   /* 0x40018000                             */
#define HT_CMP_OP1_BASE          (HT_APB0PERIPH_BASE + 0x18100)   /* 0x40018100                             */
#define HT_GPIOA_BASE            (HT_APB0PERIPH_BASE + 0x1A000)   /* 0x4001A000                             */
#define HT_GPIOB_BASE            (HT_APB0PERIPH_BASE + 0x1B000)   /* 0x4001B000                             */
#define HT_AFIO_BASE             (HT_APB0PERIPH_BASE + 0x22000)   /* 0x40022000                             */
#define HT_EXTI_BASE             (HT_APB0PERIPH_BASE + 0x24000)   /* 0x40024000                             */

/* APB1                                                                                                     */
#define HT_I2C_BASE              (HT_APB1PERIPH_BASE + 0x8000)    /* 0x40048000                             */
#define HT_WDT_BASE              (HT_APB1PERIPH_BASE + 0x28000)   /* 0x40068000                             */
#define HT_RTC_BASE              (HT_APB1PERIPH_BASE + 0x2A000)   /* 0x4006A000                             */
#define HT_PWRCU_BASE            (HT_APB1PERIPH_BASE + 0x2A100)   /* 0x4006A100                             */
#define HT_GPTM0_BASE            (HT_APB1PERIPH_BASE + 0x2E000)   /* 0x4006E000                             */
#define HT_GPTM1_BASE            (HT_APB1PERIPH_BASE + 0x2F000)   /* 0x4006F000                             */

/* AHB                                                                                                      */
#define HT_FLASH_BASE            (HT_AHBPERIPH_BASE + 0x0000)     /* 0x40080000                             */
#define HT_CKCU_BASE             (HT_AHBPERIPH_BASE + 0x8000)     /* 0x40088000                             */
#define HT_RSTCU_BASE            (HT_AHBPERIPH_BASE + 0x8100)     /* 0x40088100                             */


/**
  * @}
  */

/* Peripheral declaration                                                                                   */
#define HT_USART               ((HT_USART_TypeDef  *) HT_USART_BASE  )
#define HT_SPI                 ((HT_SPI_TypeDef    *) HT_SPI_BASE    )
#define HT_ADC                 ((HT_ADC_TypeDef    *) HT_ADC_BASE    )
#define HT_CMP_OP0             ((HT_CMP_OP_TypeDef *) HT_CMP_OP0_BASE)
#define HT_CMP_OP1             ((HT_CMP_OP_TypeDef *) HT_CMP_OP1_BASE)
#define HT_GPIOA               ((HT_GPIO_TypeDef   *) HT_GPIOA_BASE  )
#define HT_GPIOB               ((HT_GPIO_TypeDef   *) HT_GPIOB_BASE  )
#define HT_AFIO                ((HT_AFIO_TypeDef   *) HT_AFIO_BASE   )
#define HT_EXTI                ((HT_EXTI_TypeDef   *) HT_EXTI_BASE   )
#define HT_I2C                 ((HT_I2C_TypeDef    *) HT_I2C_BASE    )
#define HT_WDT                 ((HT_WDT_TypeDef    *) HT_WDT_BASE    )
#define HT_RTC                 ((HT_RTC_TypeDef    *) HT_RTC_BASE    )
#define HT_PWRCU               ((HT_PWRCU_TypeDef  *) HT_PWRCU_BASE  )
#define HT_GPTM0               ((HT_GPTM_TypeDef   *) HT_GPTM0_BASE  )
#define HT_GPTM1               ((HT_GPTM_TypeDef   *) HT_GPTM1_BASE  )
#define HT_FLASH               ((HT_FLASH_TypeDef  *) HT_FLASH_BASE  )
#define HT_CKCU                ((HT_CKCU_TypeDef   *) HT_CKCU_BASE   )
#define HT_RSTCU               ((HT_RSTCU_TypeDef  *) HT_RSTCU_BASE  )


#if defined  USE_HT32F125X_DRIVER
  #include "ht32f125x_lib.h"
#endif


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __HT32F125x_H__                                                                                  */

/**
  * @}
  */

  /**
  * @}
  */

/******************* (C) COPYRIGHT 2011 Holtek Semiconductor Inc. *****END OF FILE***                       */
