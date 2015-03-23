/****************************************Copyright (c)**************************************************
**                             
**                          company:  Witium Intelligent System Co., Ltd.
**				      上海辉度智能系统有限公司   
**                          netaddr:  www.witium.com                  
**                          TEL:      086-21-37774020
**			    FAX:      086-21-37774010
**			    Writer:   Mond Xu    
**                          mobile:   18916777010
**                          email:    mond.xu@witium.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:               Driver_----.c
** Last modified Date:      2008.12.14
** Last Version:            v3.0
** Description:            
** 
**------------------------------------------------------------------------------------------------------
** Modified by:             mond.xu
** Modified date:           2008.12.17
** Version:                 V3.0
** Description:             
**
**------------------------------------------------------------------------------------------------------
** Modified by:             mond.xu
** Modified date:           2008.12.27
** Version:                 V3.1
** Description:             
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

/********************************************************************************************************
**包含必要的头文件
*********************************************************************************************************/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"


/********************************************************************************************************
** 为驱动程序的全局变量赋初值
*********************************************************************************************************/
#if DRIVER_USART_EN > 0
	USART_TypeDef* USART_Port = USART1;
#endif

#if DRIVER_USART_EN > 0
#define  GPIO_RxPin_1                            GPIO_Pin_10
#define  GPIO_TxPin_1                            GPIO_Pin_9

#define  GPIO_RxPin_2                            GPIO_Pin_3
#define  GPIO_TxPin_2                            GPIO_Pin_2

#define  GPIO_TxPin_3							 GPIO_Pin_10
#define  GPIO_RxPin_3							 GPIO_Pin_11
#endif

#if DRIVER_USART_EN > 0
	void Driver_UsartGPIOInit(USART_TypeDef* USARTx);
	void Dirver_UsartNVICInit(USART_TypeDef* USARTx);
#endif

/*********************************************************************************************************
** Function name:           
** Descriptions:            
** input parameters:        
**                                    
** output parameters:       无      
** Returned value:          无	 
** Created By:              mond.xu
** Created date:            2008.12.14
**--------------------------------------------------------------------------------------------------------
** Modified by:            
** Modified date:           
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#if (DRIVER_USART_EN > 0)
void Driver_UsartInit(USART_TypeDef* USARTx, uint32_t uiBaudRate, uint16_t usWordLength,
						uint16_t usStopBits, uint16_t usParity)
{
	USART_InitTypeDef USART_InitStructure;

	/* USART Periph clock enable*/
	if(USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else if(USARTx == USART2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	else if(USARTx == USART3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* USARTx configuration ------------------------------------------------------*/
	USART_StructInit(&USART_InitStructure);
	/* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = uiBaudRate;
	USART_InitStructure.USART_WordLength = usWordLength;
	USART_InitStructure.USART_StopBits = usStopBits;
	USART_InitStructure.USART_Parity = usParity;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Cmd(USARTx, DISABLE);

	Driver_UsartGPIOInit(USARTx);
	Dirver_UsartNVICInit(USARTx);

	/* USART configuration */
	USART_Init(USARTx, &USART_InitStructure);
	USART_Cmd(USARTx, ENABLE);

	/* Clear transmit complete flag*/
	USART_ClearFlag(USARTx, USART_FLAG_TC);
}

void Driver_UsartGPIOInit(USART_TypeDef* USARTx)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIO Periph clock enable*/
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE );

	if(USARTx == USART1)
	{
		/* Configure USARTx_Tx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_TxPin_1;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		/* Configure USARTx_Rx as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_RxPin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if(USARTx == USART2)
	{
	    /* Configure USARTx_Tx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_TxPin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure USARTx_Rx as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_RxPin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if(USARTx == USART3)
	{
		/* Configure USARTx_Tx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_TxPin_3;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Configure USARTx_Rx as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_RxPin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
}

void Dirver_UsartNVICInit(USART_TypeDef* USARTx)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); //NVIC_PriorityGroup_0

	if(USARTx == USART1)
	{
		/* Enable USART1 IT. */
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//		USART_ClearITPendingBit(USART1, USART_IT_TXE);
//		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
//		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	}
	else if(USARTx == USART2)
	{
		/* Enable USART2 IT. */
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	}
	else if(USARTx == USART3)
	{
		/* Enable USART3 IT. */
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	}
}

void Driver_UsartSendData(USART_TypeDef* USARTx, uint8_t *pucBuf, uint32_t uiDataLength)
{
	uint32_t i = 0;

	for(i = 0; i < uiDataLength; i++)
	{
		if(pucBuf[i] != 0x00)
		{
			USART_SendData(USARTx, pucBuf[i]);
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		}
		else
			return;
	}
}

void Driver_UsartDebugPortSet(USART_TypeDef* USARTx)
{
	USART_Port = USARTx;
}

/*******************************************************************************
* Function Name  : PUTCHAR_PROTOTYPE
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
	set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Write a character to the USART */
	USART_SendData(USART_Port, (u8)ch);

	/* Loop until the end of transmission */
	while(USART_GetFlagStatus(USART_Port, USART_FLAG_TC) == RESET)
	{
	}

    return ch;
}

void Driver_USART1InterrupHandler( void )
{
	// Receive interrupt
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	    /* Read one byte from the receive data register */
	}
	// Send complete interrupt
	if( USART_GetITStatus(USART1, USART_IT_TC) != RESET )
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
	}
	// Send buffer empty interrupt
	if( USART_GetITStatus(USART1, USART_IT_TXE) != RESET )
	{
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}
#endif


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
