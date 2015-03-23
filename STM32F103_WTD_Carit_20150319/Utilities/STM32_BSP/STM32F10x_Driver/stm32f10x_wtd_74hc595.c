#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

#define DO_RCL_PORT			s74hc595->RCL_GPIOx
#define DO_SDA_PORT			s74hc595->SDA_GPIOx
#define DO_SCL_PORT			s74hc595->SCL_GPIOx

#define DO_RCL_PIN			s74hc595->RCL_GPIO_Pin
#define DO_SDA_PIN			s74hc595->SDA_GPIO_Pin
#define DO_SCL_PIN			s74hc595->SCL_GPIO_Pin

#define RCL_ON	{GPIO_SetBits(DO_RCL_PORT,DO_RCL_PIN);}
#define RCL_OFF	{GPIO_ResetBits(DO_RCL_PORT,DO_RCL_PIN);}
#define SDA_ON	{GPIO_SetBits(DO_SDA_PORT,DO_SDA_PIN);}
#define SDA_OFF	{GPIO_ResetBits(DO_SDA_PORT,DO_SDA_PIN);}
#define SCL_ON	{GPIO_SetBits(DO_SCL_PORT,DO_SCL_PIN);}
#define SCL_OFF	{GPIO_ResetBits(DO_SCL_PORT,DO_SCL_PIN);}

void Driver_74hc595_Init(S74HC595TYPE* s74hc595)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO RCC */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
		
	/* Set AF mode remap config*/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 

	GPIO_InitStructure.GPIO_Pin = DO_RCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DO_RCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DO_SDA_PIN;
	GPIO_Init(DO_SDA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DO_SCL_PIN;
	GPIO_Init(DO_SCL_PORT, &GPIO_InitStructure);
	
	RCL_OFF
	SDA_OFF
	SCL_OFF
}

void Driver_74hc595_Ctrl(S74HC595TYPE* s74hc595, uint16_t usState)
{
	uint8_t i = 0;

	RCL_OFF
	SDA_OFF
	SCL_OFF

	for(i = 0x00; i < 16; i ++)
	{
		if(usState & (1 << (15 - i)))
		{
			SDA_ON
		}
		else
		{
			SDA_OFF
		}
		SCL_ON
		SCL_OFF
	}
	RCL_ON
	RCL_OFF
}

