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
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "port.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define USART1_ENABLED         ( 1 )
#define USART2_ENABLED         ( 1 )
#define USART3_ENABLED         ( 1 )

#define USART_IDX_LAST         ( USART1_ENABLED + USART2_ENABLED + USART3_ENABLED )

#define USART_INVALID_PORT     ( 0xFF )

/* ----------------------- Static variables ---------------------------------*/
#if USART1_ENABLED == 1
#define  USART1_RxPin        GPIO_Pin_10 //GPA
#define  USART1_TxPin        GPIO_Pin_9
#define  USART1_EnPin			GPIO_Pin_1
#endif

#if USART2_ENABLED == 1
#define  USART2_RxPin        GPIO_Pin_3  //GPA
#define  USART2_TxPin        GPIO_Pin_2
#define  USART2_EnPin			GPIO_Pin_1
#endif

#if USART3_ENABLED == 1
#define  USART3_RxPin        GPIO_Pin_10 //GPB
#define  USART3_TxPin        GPIO_Pin_11
#define  USART3_EnPin			GPIO_Pin_10 //GPC
#endif

const struct xUSARTHWMappings_t
{
	USART_TypeDef	*pUsart;
	GPIO_TypeDef	*pGpioRx;
	GPIO_TypeDef	*pGpioTx;
	GPIO_TypeDef	*pGpioEn;
	uint16_t		xUSARTRxPin;
	uint16_t		xUSARTTxPin;
	uint16_t		xUSARTEnPin;
	uint16_t		xUSARTReserve;
} xUSARTHWMappings[] =
{
#if USART1_ENABLED == 1
    {USART1, GPIOA, GPIOA, GPIOA, USART1_RxPin, USART1_TxPin, USART1_EnPin},
#endif
#if USART2_ENABLED == 1
    {USART2, GPIOA, GPIOA, GPIOA, USART2_RxPin, USART2_TxPin, USART2_EnPin},
#endif
#if USART3_ENABLED == 1
    {USART3, GPIOB, GPIOB, GPIOC, USART3_RxPin, USART3_TxPin, USART3_EnPin},
#endif
};

static UCHAR    ucUsedPort = USART_INVALID_PORT;

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bStatus = FALSE;
    USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_StructInit(&USART_InitStructure);
    if( (ucPORT <= USART_IDX_LAST) && (ucPORT > 0) && (ucUsedPort == USART_INVALID_PORT) )
    {
        bStatus = TRUE;
        switch ( eParity )
        {
        case MB_PAR_NONE:
            USART_InitStructure.USART_Parity = USART_Parity_No;
			USART_InitStructure.USART_StopBits = USART_StopBits_2;
            break;
        case MB_PAR_ODD:
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
            USART_InitStructure.USART_StopBits = USART_StopBits_1;     
			    break;
        case MB_PAR_EVEN:
            USART_InitStructure.USART_Parity = USART_Parity_Even;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
            break;
        default:
            bStatus = FALSE;
            break;
        }

        switch ( ucDataBits )
        {
        case 8:
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            break;
        case 7:// STM32 字长为7时必须把8位其中一位作为校验位
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            break;
        default:
            bStatus = FALSE;
        }

		USART_InitStructure.USART_BaudRate = ulBaudRate;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        
        if( TRUE == bStatus )
        {
            ucUsedPort = (ucPORT - 1); // 端口号和对应数组编号相差1

			USART_Cmd(xUSARTHWMappings[ucUsedPort].pUsart, DISABLE);

			//GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

            // GPIO configure
            /* Configure USARTx_Tx as alternate function push-pull */
            GPIO_InitStructure.GPIO_Pin = xUSARTHWMappings[ucUsedPort].xUSARTTxPin;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(xUSARTHWMappings[ucUsedPort].pGpioTx, &GPIO_InitStructure);

            /* Configure USARTx_Rx as input floating */
            GPIO_InitStructure.GPIO_Pin = xUSARTHWMappings[ucUsedPort].xUSARTRxPin;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(xUSARTHWMappings[ucUsedPort].pGpioRx, &GPIO_InitStructure);
	         
		    /* Configure RS485 Tx Enable Pin */
		    GPIO_InitStructure.GPIO_Pin = xUSARTHWMappings[ucUsedPort].xUSARTEnPin;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_Init(xUSARTHWMappings[ucUsedPort].pGpioEn, &GPIO_InitStructure);
            GPIO_ResetBits(xUSARTHWMappings[ucUsedPort].pGpioEn, xUSARTHWMappings[ucUsedPort].xUSARTEnPin);

			/* Enable USART2 IT. */
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init( &NVIC_InitStructure );

		    /* Configure Clock */
		    if(xUSARTHWMappings[ucUsedPort].pUsart == USART1)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			} 
			 else if(xUSARTHWMappings[ucUsedPort].pUsart == USART2)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			}
		    else if(xUSARTHWMappings[ucUsedPort].pUsart == USART3)
			{
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			}

			USART_Init(xUSARTHWMappings[ucUsedPort].pUsart, &USART_InitStructure);
			/* Enable USART */
			USART_Cmd(xUSARTHWMappings[ucUsedPort].pUsart, ENABLE);
        }
    }
//vMBPortSerialEnable(TRUE, FALSE);

	return bStatus;
}

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if( xRxEnable )
	{
		if( 0 != xUSARTHWMappings[ucUsedPort].xUSARTEnPin )
		{
			GPIO_ResetBits(xUSARTHWMappings[ucUsedPort].pGpioEn, xUSARTHWMappings[ucUsedPort].xUSARTEnPin);
		}
		
		USART_ClearFlag(xUSARTHWMappings[ucUsedPort].pUsart,USART_FLAG_RXNE);
		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_RXNE);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_RXNE, ENABLE);
	}
	else
	{
		USART_ClearFlag(xUSARTHWMappings[ucUsedPort].pUsart,USART_FLAG_RXNE);
		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_RXNE);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_RXNE, DISABLE);
	}

	if( xTxEnable )
	{
		if( 0 != xUSARTHWMappings[ucUsedPort].xUSARTEnPin )
		{
			GPIO_SetBits(xUSARTHWMappings[ucUsedPort].pGpioEn, xUSARTHWMappings[ucUsedPort].xUSARTEnPin);
		}

		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TXE);
		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TC);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TXE, DISABLE);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TC, ENABLE);

		pxMBFrameCBTransmitterEmpty(  );// 启动第一次发送，这样才可以进入发送完成中断
	}
	else
	{
		if( 0 != xUSARTHWMappings[ucUsedPort].xUSARTEnPin )
		{
			GPIO_ResetBits(xUSARTHWMappings[ucUsedPort].pGpioEn, xUSARTHWMappings[ucUsedPort].xUSARTEnPin);
		}

		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TXE);
		USART_ClearITPendingBit(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TC);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TXE, ENABLE);
		USART_ITConfig(xUSARTHWMappings[ucUsedPort].pUsart, USART_IT_TC, DISABLE);
	}
}

void
vMBPortSerialClose( void )
{
	if( USART_INVALID_PORT != ucUsedPort )
	{
		vMBPortSerialEnable(FALSE, FALSE);

		/* Configure Clock Disable */
		if(xUSARTHWMappings[ucUsedPort].pUsart == USART1)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
		} 
		else if(xUSARTHWMappings[ucUsedPort].pUsart == USART2)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
		}
		else if(xUSARTHWMappings[ucUsedPort].pUsart == USART3)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);
		}

		/* Disable USART */
		USART_Cmd(xUSARTHWMappings[ucUsedPort].pUsart, DISABLE);

		ucUsedPort = USART_INVALID_PORT;
	}
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	USART_SendData(xUSARTHWMappings[ucUsedPort].pUsart, ucByte);
	return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = (uint8_t)USART_ReceiveData(xUSARTHWMappings[ucUsedPort].pUsart);
    return TRUE;
}

void
prvvMBSerialIRQHandler( void )
{
	BOOL            bTaskWoken = FALSE;

	vMBPortSetWithinException( TRUE );

	// Recived interrupt
	if( USART_GetITStatus(USART2, USART_IT_RXNE) != RESET )
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
#if MB_MASTER_ENABLED > 0
		vMBPortWaitEnd();
#endif
		bTaskWoken = pxMBFrameCBByteReceived(  );
	}
	// Send complete interrupt
	if( USART_GetITStatus(USART2, USART_IT_TC) != RESET )
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		bTaskWoken = pxMBFrameCBTransmitterEmpty(  );
	}
	// Send buffer empty interrupt
	if( USART_GetITStatus(USART2, USART_IT_TXE) != RESET )
	{
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	}

	vMBPortSetWithinException( FALSE );

	portEND_SWITCHING_ISR( bTaskWoken ? pdTRUE : pdFALSE );
}
