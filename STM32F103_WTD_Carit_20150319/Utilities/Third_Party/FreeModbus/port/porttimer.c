/*
 * FreeModbus Libary: ST STM32F103 Demo Application
 * Copyright (C) 2014 Carit Zhu <carit.zhu@witium.com>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portserial.c,v 1.1 2014/08/20 10:02:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_TIMER_DEBUG							( 0 )
#define MB_TIMER_DEV							( TIM2 ) 
#define MB_TIMER_PRESCALER						( 359UL ) // 359 + 1 = 360 Timer Internal Clock is 72MHz

/* Timer ticks are counted in multiples of 50us. Therefore 20000 ticks are
 * one second.
 */
#define MB_TIMER50US_TICKS						( 20000UL )

/* The highest available interrupt priority. */
#define timerHIGHEST_PRIORITY					( 0 )

#if MB_TIMER_DEBUG == 1

#endif

/* ----------------------- Static variables ---------------------------------*/
#if MB_TIMER_DEBUG == 1

#endif
#if MB_MASTER_ENABLED > 0
static ULONG ulMasterTimeoutMs = 0;
#endif

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us
#if MB_MASTER_ENABLED > 0
                 , ULONG ulTimeoutMs
#endif
                 )
{
	USHORT   usTimerDeltaOCRA;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

#if MB_TIMER_DEBUG == 1
#endif

	usTimerDeltaOCRA =
		( ( configCPU_CLOCK_HZ / ( MB_TIMER_PRESCALER + 1 ) ) *
		usTim1Timerout50us ) / ( MB_TIMER50US_TICKS );

	/* Timer use internal clock */
	TIM_InternalClockConfig(MB_TIMER_DEV);

	/* Timer clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* TIM2 default */
	TIM_DeInit(MB_TIMER_DEV);

	// TIM2 configuration for 10ms
	if( usTimerDeltaOCRA > 0 )
	{
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Period = usTimerDeltaOCRA; //		1 tick = 5us
		TIM_TimeBaseStructure.TIM_Prescaler = MB_TIMER_PRESCALER;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0; //
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //
		TIM_TimeBaseInit(MB_TIMER_DEV, &TIM_TimeBaseStructure);
		TIM_ARRPreloadConfig( MB_TIMER_DEV, ENABLE );
		TIM_UpdateRequestConfig(MB_TIMER_DEV, TIM_UpdateSource_Global);
	}

	/* Enable TIM2 IT.  TIM3 does not generate an interrupt. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = timerHIGHEST_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	// Clear TIM2 update pending flag
	//TIM_ClearFlag(MB_TIMER_DEV, TIM_FLAG_Update);

	// Enable TIM2 Update interrupt
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	  
#if MB_MASTER_ENABLED > 0
	ulMasterTimeoutMs = ulTimeoutMs;
#endif

	return TRUE;
}

void
vMBPortTimerClose( void )
{
	/* TIM2 disable counter */
	TIM_Cmd(MB_TIMER_DEV, DISABLE);

	// Clear TIM2 update pending flag
	TIM_ClearFlag(MB_TIMER_DEV, TIM_FLAG_Update);

	// Disable TIM2 Update interrupt
	TIM_ITConfig(MB_TIMER_DEV, TIM_IT_Update, DISABLE); 

	// Disable Timer Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}

void
vMBPortTimersEnable(  )
{
#if MB_TIMER_DEBUG == 1
#endif
	// Clear TIM2 counter register
	TIM_SetCounter(MB_TIMER_DEV, 0);

	// Clear TIM2 update pending flag
	TIM_ClearFlag(MB_TIMER_DEV, TIM_FLAG_Update);

	// Enable TIM2 Update interrupt
	TIM_ITConfig(MB_TIMER_DEV, TIM_IT_Update, ENABLE);

	// TIM2 enable counter
	TIM_Cmd(MB_TIMER_DEV, ENABLE); 
}

void
vMBPortTimersDisable(  )
{
	// Clear TIM2 update pending flag
	TIM_ClearFlag(MB_TIMER_DEV, TIM_FLAG_Update);

	// Disable TIM2 Update interrupt
	TIM_ITConfig(MB_TIMER_DEV, TIM_IT_Update, DISABLE); 

	/* TIM2 disable counter */
	TIM_Cmd(MB_TIMER_DEV, DISABLE);

#if MB_TIMER_DEBUG == 1
#endif
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
    vTaskDelay( usTimeOutMS / portTICK_RATE_MS );
}

#if MB_MASTER_ENABLED > 0
BOOL
bMBPortTimerWaitEnable( void )
{
	return vMBPortWaitStart(ulMasterTimeoutMs / portTICK_RATE_MS);
}
#endif 

void
prvvMBTimerIRQHandler( void )
{
	BOOL            bTaskWoken = FALSE;

	vMBPortSetWithinException( TRUE );

	if( TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET )
	{
#if MB_TIMER_DEBUG == 1
#endif
		( void )pxMBPortCBTimerExpired(  );

		/* Clear Interrupt */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	vMBPortSetWithinException( FALSE );

	portEND_SWITCHING_ISR( bTaskWoken ? pdTRUE : pdFALSE );
}
