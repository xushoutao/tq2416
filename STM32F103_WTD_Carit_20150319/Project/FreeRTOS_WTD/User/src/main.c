/*
    STM32F103 WTD V1.1.0 - Copyright (C) 2014 Witium Ltd.
    All rights reserved

    VISIT http://www.witium.com TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    STM32F103 WTD provides                                             *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the WTD project.

     1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?                                                  *
     *                                                                       *
     *    http://www.witium.com                                              *
     *                                                                       *
    ***************************************************************************
*/

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Modbus Polling" task - 
 *
 * "Polling" task - 
 *
 */

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
					 
/* User app includes. */
#include "stm32f10x_wtd_drivers.h"
#include "wtdconf.h"
#include "mb.h"

/* Task priorities. */
#define mainMODBUS_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )
#define mainPOLL_TASK_PRIORITY           	( tskIDLE_PRIORITY )

/* Task stack size */
/* The modbus task uses the sprintf function so requires a little more stack. */
#define mainMODBUS_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 128 )
#define mainPOLL_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 128 )

/*----------------------Private Functions---------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware( void );

/*
 * Configure WTD Devices.
 */
static void vDevice_Init(void);

/*
 * Checks the status of all the demo tasks then prints a message to the
 * display.  The message will be either PASS - and include in brackets the
 * maximum measured jitter time (as described at the to of the file), or a
 * message that describes which of the standard demo tasks an error has been
 * discovered in.
 *
 * Messages are not written directly to the terminal, but passed to vLCDTask
 * via a queue.
 */
static void vModbusTask( void *pvParameters );
static void vPollTask( void *pvParameters );
//static void vSimpleLinkTask( void *pvParameters );

/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
//extern void vPortTimerInit( void );

/*-----------------------------------------------------------*/

int main( void )
{
#ifdef DEBUG
	debug();
#endif

	/* Setup hardware */
	prvSetupHardware();

	/* Init device function */
	vDevice_Init();

	/* Init modbus protocol stack */
	//eMBInit(MB_RTU, 0x0A, 2, 115200, MB_PAR_NONE);
	eMBInit(MB_RTU, 0x0A, 2, 115200, MB_PAR_NONE, 500);

	/* Start the standard tasks. */

	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate( vModbusTask, "MBPollTask", mainMODBUS_TASK_STACK_SIZE, NULL, mainMODBUS_TASK_PRIORITY, NULL );
	xTaskCreate( vPollTask, "PollTask", mainPOLL_TASK_STACK_SIZE, NULL, mainPOLL_TASK_PRIORITY, NULL );

	/* The suicide tasks must be created last as they need to know how many
	tasks were running prior to their creation in order to ascertain whether
	or not the correct/expected number of tasks are running at any given time. */
	//vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the
	idle task. */
	/* We should never get here as control is now taken by the scheduler */
  	for( ;; );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
void Dummy_IntHandler(void)
{
	// Receive interrupt
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	    /* Read one byte from the receive data register */
	}
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}
	//RCC_WaitForHSEStartUp();

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	/* Init debug serial port */
	Driver_UsartInit(USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No);
	Driver_UsartDebugPortSet(USART1);
	
	/* Relocate Interrupt Vector Table */
	Driver_IntRelocate();
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
void vModbusTask( void *pvParameters )
{
//	USHORT			usRegData = 0;
//	eMBErrorCode    eStatus;
	for( ;; )
	{
#if 0
		eStatus = eModbus_Read_Holding_Register(0x01, 0x0001, 0x0001, &usRegData);
		if (eStatus != MB_ENOERR)
		{
			printf("Error: Read_Holding_Register error code is %d.\r\n", eStatus);	
		}
		else
		{
			printf("Read_Holding_Register data is %d.\r\n", usRegData);
		}
#endif
#if 1
		if( MB_ENOERR != eMBEnable(  ) )
		{
			/* Enable failed. */
		}
		else
		{
			//usRegHoldingBuf[0] = 1;
			do
			{
				( void )eMBPoll(  );

				/* Here we simply count the number of poll cycles. */
				//    usRegInputBuf[0]++;
			}while( TRUE );
		}
		( void )eMBDisable(  );
		( void )eMBClose(  );
#endif
		vTaskDelay(250);
	}
}

void vPollTask( void *pvParameters )
{
	for( ;; )
	{
		/* Display the message.  Print each message to a different position. */
		//printf( "Task2\r\n" );
		vTaskDelay(250);
	}
}

void vDevice_Init(void)
{
	E2P_Init();

#if WTD_RELAY_EN
	if_relay_init();
#endif
}

/*-----------------------------------------------------------*/

#ifdef  DEBUG
/* Keep the linker happy. */
void assert_failed( unsigned char* pcFile, unsigned long ulLine )
{
	for( ;; )
	{
	}
}
#endif
