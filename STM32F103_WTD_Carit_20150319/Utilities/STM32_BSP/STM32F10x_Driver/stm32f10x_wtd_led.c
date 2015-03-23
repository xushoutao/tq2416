/****************************************Copyright (c)**************************************************
**                             
**                          company:  Witium Intelligent System Co., Ltd.
**				      �Ϻ��Զ�����ϵͳ���޹�˾   
**                          netaddr:  www.witium.com                  
**                          TEL:      086-21-37774020
**			    FAX:      086-21-37774010
**			    Writer:   Mond Xu    
**                          mobile:   18916777010
**                          email:    mond.xu@witium.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:               stm32f10x_wtd_led.c
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
**������Ҫ��ͷ�ļ�
*********************************************************************************************************/
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"


/********************************************************************************************************
** Ϊ��������ľֲ���������ֵ
*********************************************************************************************************/
#if DRIVER_LED_EN > 0
	S74HC595TYPE l_stLed;
#endif

#if DRIVER_LED_EN > 0
#define  GPIO_RCL_Pin					GPIO_Pin_8
#define  GPIO_RCL_Port					GPIOA

#define  GPIO_SCL_Pin					GPIO_Pin_3
#define  GPIO_SCL_Port					GPIOB

#define  GPIO_SDA_Pin					GPIO_Pin_9
#define  GPIO_SDA_Port					GPIOC
#endif

#if DRIVER_USART_EN > 0
#endif

/*********************************************************************************************************
** Function name:           
** Descriptions:            
** input parameters:        
**                                    
** output parameters:       ��      
** Returned value:          ��	 
** Created By:              mond.xu
** Created date:            2008.12.14
**--------------------------------------------------------------------------------------------------------
** Modified by:            
** Modified date:           
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#if (DRIVER_USART_EN > 0)
void Driver_LedInit(void)
{
	l_stLed.RCL_GPIO_Pin = GPIO_RCL_Pin;
	l_stLed.SCL_GPIO_Pin = GPIO_SCL_Pin;
	l_stLed.SDA_GPIO_Pin = GPIO_SDA_Pin;

	l_stLed.RCL_GPIOx = GPIO_RCL_Port;
	l_stLed.SCL_GPIOx = GPIO_SCL_Port;
	l_stLed.SDA_GPIOx = GPIO_SDA_Port;

	Driver_74hc595_Init(&l_stLed);
}

void Driver_LedSToPCtrl(uint16_t usLedState)
{
	Driver_74hc595_Ctrl(&l_stLed, usLedState);
}

#endif


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
