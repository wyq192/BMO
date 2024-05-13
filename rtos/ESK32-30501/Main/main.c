/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    main.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "ht32.h"
#include "console.h"

static CKCU_PeripClockConfig_TypeDef CKCUClock;
static USART_InitTypeDef USARTInit;
static TM_TimeBaseInitTypeDef TimeInit;
static TM_OutputInitTypeDef OutInit;
static I2C_InitTypeDef I2CInit;
static SPI_InitTypeDef SPIInit;
static TM_CaptureInitTypeDef CapInit;

/* Track how many times tick interrupt has occurred. */
static unsigned int uTickInterruptCounter = 0;

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware( void );

/*
 * @brief   Application entry point.
 */
int main(void)
{
	prvSetupHardware();
    
    StartTasks(tskIDLE_PRIORITY + 2);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
	return 0;
}

static void prvSetupHardware( void )
{
    // USART1 for Console =====================================================
    // PA4: USART1_TX
    // PA5: USART1_RX
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
    CKCUClock.Bit.PA = 1;
    CKCUClock.Bit.USART1 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);

    GPIO_PullResistorConfig(HT_GPIOA, GPIO_PIN_5, GPIO_PR_UP);
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);

    USARTInit.USART_BaudRate = 115200;
    USARTInit.USART_WordLength = USART_WORDLENGTH_8B;
    USARTInit.USART_StopBits = USART_STOPBITS_1;
    USARTInit.USART_Parity = USART_PARITY_NO;
    USARTInit.USART_Mode = USART_MODE_NORMAL;
    USART_Init(HT_USART1, &USARTInit);

    USART_TxCmd(HT_USART1, ENABLE);
    USART_RxCmd(HT_USART1, ENABLE);
    
#if 0
    // Step Motor: ULN2003A ===================================================
    // PC11/PC12/PC14/PC15
    memset(&CKCUClock, 0, sizeof(CKCUClock));
	CKCUClock.Bit.PC = 1;
    CKCUClock.Bit.AFIO = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_11 | AFIO_PIN_12 | AFIO_PIN_14 | AFIO_PIN_15, AFIO_FUN_GPIO);
    GPIO_DirectionConfig(HT_GPIOC, AFIO_PIN_11 | AFIO_PIN_12 | AFIO_PIN_14 | AFIO_PIN_15, GPIO_DIR_OUT);
#endif

    // LED on board ===========================================================
    // PC14/PC15: GPIO
    memset(&CKCUClock, 0, sizeof(CKCUClock));
	CKCUClock.Bit.PC = 1;
    CKCUClock.Bit.AFIO = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_14 | AFIO_PIN_15, AFIO_FUN_GPIO);
    GPIO_DirectionConfig(HT_GPIOC, AFIO_PIN_14 | AFIO_PIN_15, GPIO_DIR_OUT);
    
    // Buzzer =================================================================
    // PA15: SCTM0, Channel 1
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
    CKCUClock.Bit.SCTM1 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);

    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_15, AFIO_FUN_SCTM1);

    int reload = SystemCoreClock / 4 / 3000 - 1;        // default CRR value. will be overridden later.
    int compare = (reload + 1) * (100 - 50) / 100;      // default count value for duty. will be overridden later.
    memset(&TimeInit, 0, sizeof(TimeInit));
    TimeInit.CounterReload = reload;
    TimeInit.Prescaler = 3;         // prescaler = (3 + 1) = 4
    TimeInit.RepetitionCounter = 0;
    TimeInit.CounterMode = TM_CNT_MODE_UP;
    TimeInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
    TM_TimeBaseInit(HT_SCTM1, &TimeInit);

    memset(&OutInit, 0, sizeof(OutInit));
    OutInit.Channel = TM_CH_0;
    OutInit.OutputMode = TM_OM_PWM2;
    OutInit.Control = TM_CHCTL_ENABLE;
    OutInit.ControlN = TM_CHCTL_DISABLE;
    OutInit.Polarity = TM_CHP_NONINVERTED;
    OutInit.PolarityN = TM_CHP_NONINVERTED;
    OutInit.IdleState = MCTM_OIS_LOW;
    OutInit.IdleStateN = MCTM_OIS_HIGH;
    OutInit.Compare = compare;
    TM_OutputInit(HT_SCTM1, &OutInit);
    
    TM_ChannelConfig(HT_SCTM1, TM_CH_0, TM_CHCTL_DISABLE);
    TM_Cmd(HT_SCTM1, ENABLE);
#if 0
    TM_ClearFlag(HT_MCTM0, TM_FLAG_UEV);
    /* BUZZER TM Channel Main Output enable                                                                   */
    MCTM_CHMOECmd(HT_MCTM0, ENABLE);
    NVIC_EnableIRQ(MCTM0_IRQn);
#endif
    
    // LCD Display ============================================================
    // PA0: I2C1_SCL
    // PA1: I2C1_SDA
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
    CKCUClock.Bit.PA  = 1;
	CKCUClock.Bit.PC = 1;
	CKCUClock.Bit.I2C1 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
  
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_0 | AFIO_PIN_1 , AFIO_FUN_I2C);
    
    I2CInit.I2C_GeneralCall = DISABLE;
    I2CInit.I2C_AddressingMode = I2C_ADDRESSING_7BIT;
    I2CInit.I2C_Acknowledge = DISABLE;
    I2CInit.I2C_OwnAddress = I2C_MASTER_ADDRESS;
    I2CInit.I2C_Speed = ClockSpeed;
    I2C_Init(HT_I2C1, &I2CInit);
	I2C_Cmd(HT_I2C1 , ENABLE);

    // RFID ==================================================================
    // PB1: GPIO; used for RFID RST
    // PB2: GPIO; CS
    // PB3: SPI0_SCK
    // PB4: SPI0_MOSI
    // PB5: SPI0_MISO
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
	CKCUClock.Bit.PB = 1;
	CKCUClock.Bit.SPI0 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);

	AFIO_GPxConfig(GPIO_PB, AFIO_PIN_1, AFIO_FUN_GPIO);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_2, AFIO_FUN_GPIO);
	AFIO_GPxConfig(GPIO_PB, AFIO_PIN_3, AFIO_FUN_SPI);
	AFIO_GPxConfig(GPIO_PB, AFIO_PIN_4, AFIO_FUN_SPI);
	AFIO_GPxConfig(GPIO_PB, AFIO_PIN_5, AFIO_FUN_SPI);
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_1, GPIO_DIR_OUT);
    GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_2, GPIO_DIR_OUT);

    SPIInit.SPI_Mode = SPI_MASTER;
    SPIInit.SPI_FIFO = SPI_FIFO_DISABLE;
    SPIInit.SPI_DataLength = SPI_DATALENGTH_8;
    SPIInit.SPI_SELMode = SPI_SEL_SOFTWARE;
    SPIInit.SPI_SELPolarity = SPI_SELPOLARITY_LOW;
    SPIInit.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    SPIInit.SPI_CPOL = SPI_CPOL_LOW;
    SPIInit.SPI_CPHA = SPI_CPHA_FIRST;
    SPIInit.SPI_RxFIFOTriggerLevel = 0;
    SPIInit.SPI_TxFIFOTriggerLevel = 0;
    SPIInit.SPI_ClockPrescaler = 48;
    SPI_Init(HT_SPI0, &SPIInit);

    SPI_SELOutputCmd(HT_SPI0, ENABLE);
    SPI_Cmd(HT_SPI0, ENABLE);

    // 4x4 Keypad =============================================================
    // PA3, PA2, PA10, PA9 => Row 1 (leftmost), 2, 3, 4         // Input with internal pull high.
    // PD0, PD1, PD2, PD3 => Col 1, 2, 3, 4 (rightmost)     // Output
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.PA = 1;
	CKCUClock.Bit.PD = 1;
    CKCUClock.Bit.AFIO = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);

	AFIO_GPxConfig(GPIO_PA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_9, AFIO_FUN_GPIO);	
	AFIO_GPxConfig(GPIO_PD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, AFIO_FUN_GPIO);

	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_9, GPIO_DIR_IN);
	GPIO_DirectionConfig(HT_GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_OUT);

	GPIO_PullResistorConfig(HT_GPIOA, GPIO_PIN_3 | GPIO_PIN_2 |	GPIO_PIN_10 | GPIO_PIN_9, GPIO_PR_UP);	
	GPIO_PullResistorConfig(HT_GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PR_DISABLE);	
    
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_9, ENABLE);
	//GPIO_InputConfig(HT_GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, ENABLE);

    // Micro Servo SG90 ========================================================
    // PC1: MCTM0 Channel 0
    // PB6: MCTM0 Channel 2
    // PB8: MCTM0 Channel 3
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
    CKCUClock.Bit.PB = 1;
    CKCUClock.Bit.PC = 1;
	CKCUClock.Bit.MCTM0 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1, AFIO_FUN_MCTM_GPTM);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_6, AFIO_FUN_MCTM_GPTM);
    AFIO_GPxConfig(GPIO_PB, AFIO_PIN_8, AFIO_FUN_MCTM_GPTM);

    memset(&TimeInit, 0, sizeof(TimeInit));
    int period = SystemCoreClock / 0x10000;
    TimeInit.CounterReload = period - 1;
    int div = 0x10000 / 50;
    TimeInit.Prescaler = div - 1;   // frequency = 50Hz
    TimeInit.CounterMode = TM_CNT_MODE_UP;
    TimeInit.PSCReloadTime = TM_PSC_RLD_UPDATE;     // TM_PSC_RLD_IMMEDIATE
    TM_TimeBaseInit(HT_MCTM0, &TimeInit);
    
    // Infrared ================================================================
    // PC5: GPTM0 Channel 1
    memset(&CKCUClock, 0, sizeof(CKCUClock));
	CKCUClock.Bit.GPTM0 = 1;
    CKCUClock.Bit.AFIO = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_5, AFIO_FUN_MCTM_GPTM);
    
    memset(&TimeInit, 0, sizeof(TimeInit));
    TimeInit.Prescaler = 1 - 1;                         // Timer clock = CK_AHB / 1
    TimeInit.CounterReload = 0xffff;
    TimeInit.RepetitionCounter = 0;
    TimeInit.CounterMode = TM_CNT_MODE_UP;
    TimeInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
    TM_TimeBaseInit(HT_GPTM0, &TimeInit);

    // Clear Update Event Interrupt flag since the "TM_TimeBaseInit()" writes the UEV1G bit
    TM_ClearFlag(HT_GPTM0, TM_FLAG_UEV);

    TM_CaptureStructInit(&CapInit);
    CapInit.Channel = TM_CH_1;
    CapInit.Polarity = TM_CHP_NONINVERTED;
    CapInit.Selection = TM_CHCCS_DIRECT;
    CapInit.Prescaler = TM_CHPSC_OFF;
    CapInit.Filter = 0x0;
    TM_CaptureInit(HT_GPTM0, &CapInit);

    // Enable TM Channel Capture and Update Event interrupts
    TM_IntConfig(HT_GPTM0, TM_INT_CH1CC | TM_INT_UEV, ENABLE);
    NVIC_EnableIRQ(GPTM0_IRQn);
    TM_Cmd(HT_GPTM0, ENABLE);
    
    #if 1
    // Ultrasonic Sensor =======================================================
    // PC10: GPTM1 Channel 0; Echo
    // PA11: SCTM0 Channel 0; Ultrasonic Trigger
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.AFIO = 1;
	//CKCUClock.Bit.PC = 1;
	CKCUClock.Bit.GPTM1 = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PC, AFIO_PIN_10, AFIO_FUN_MCTM_GPTM);
    
    memset(&TimeInit, 0, sizeof(TimeInit));
    TimeInit.Prescaler = 1 - 1;                         // Timer clock = CK_AHB / 1
    TimeInit.CounterReload = 0xffff;
    TimeInit.RepetitionCounter = 0;
    TimeInit.CounterMode = TM_CNT_MODE_UP;
    TimeInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
    TM_TimeBaseInit(HT_GPTM1, &TimeInit);

    // Clear Update Event Interrupt flag since the "TM_TimeBaseInit()" writes the UEV1G bit
    TM_ClearFlag(HT_GPTM1, TM_FLAG_UEV);

    memset(&CapInit, 0, sizeof(CapInit));
    TM_CaptureStructInit(&CapInit);
    CapInit.Channel = TM_CH_0;
    CapInit.Polarity = TM_CHP_NONINVERTED;
    CapInit.Selection = TM_CHCCS_DIRECT;
    CapInit.Prescaler = TM_CHPSC_OFF;
    CapInit.Filter = 0x0;
    TM_CaptureInit(HT_GPTM1, &CapInit);

    // Enable TM Channel Capture and Update Event interrupts
    TM_IntConfig(HT_GPTM1, TM_INT_CH0CC | TM_INT_UEV, ENABLE);
    NVIC_EnableIRQ(GPTM1_IRQn);
    TM_Cmd(HT_GPTM1, ENABLE);
    
    memset(&CKCUClock, 0, sizeof(CKCUClock));
    CKCUClock.Bit.SCTM0 = 1;
    CKCUClock.Bit.AFIO = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    AFIO_GPxConfig(GPIO_PA, AFIO_PIN_11, AFIO_FUN_SCTM0);

// Timer clock 4kHz
#define PWM_TM_PCLK     4000

    memset(&TimeInit, 0, sizeof(TimeInit));
    TimeInit.Prescaler = (SystemCoreClock / PWM_TM_PCLK) - 1;
    TimeInit.CounterReload = PWM_TM_PCLK - 1;                  // PWM frequency = 1 Hz
    TimeInit.RepetitionCounter = 0;
    TimeInit.CounterMode = TM_CNT_MODE_UP;
    TimeInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
    TM_TimeBaseInit(HT_SCTM0, &TimeInit);

    memset(&OutInit, 0, sizeof(OutInit));
    OutInit.Channel = TM_CH_0;
    OutInit.OutputMode = TM_OM_PWM2;
    OutInit.Control = TM_CHCTL_ENABLE;
    OutInit.ControlN = TM_CHCTL_DISABLE;
    OutInit.Polarity = TM_CHP_NONINVERTED;
    OutInit.PolarityN = TM_CHP_NONINVERTED;
    OutInit.IdleState = MCTM_OIS_LOW;
    OutInit.IdleStateN = MCTM_OIS_HIGH;
    OutInit.Compare =  PWM_TM_PCLK - (PWM_TM_PCLK * 0.0025);      // PWM CCR = PWM clock * 2500 us
    OutInit.AsymmetricCompare = 0;
    TM_OutputInit(HT_SCTM0, &OutInit);

    TM_Cmd(HT_SCTM0, ENABLE);
    #endif
}

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
#if mainCHECK_INTERRUPT_STACK == 1
extern unsigned long _pvHeapStart[];

	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */

	/* Manually check the last few bytes of the interrupt stack to check they
	have not been overwritten.  Note - the task stacks are automatically
	checked for overflow if configCHECK_FOR_STACK_OVERFLOW is set to 1 or 2
	in FreeRTOSConifg.h, but the interrupt stack is not. */
	configASSERT( memcmp( ( void * ) _pvHeapStart, ucExpectedInterruptStackValues, sizeof( ucExpectedInterruptStackValues ) ) == 0U );
#endif /* mainCHECK_INTERRUPT_STACK */

	uTickInterruptCounter++;
}


