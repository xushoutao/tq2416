#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

//并转串IO定义
#define P2S_SDO_PORT		s74hc165->SDO_GPIOx
#define P2S_LD_PORT			s74hc165->LD_GPIOx
#define P2S_CLK_PORT		s74hc165->CLK_GPIOx

#define P2S_SDO_PIN			s74hc165->SDO_GPIO_PIN
#define P2S_LD_PIN			s74hc165->LD_GPIO_Pin
#define P2S_CLK_PIN			s74hc165->CLK_GPIO_Pin

#define P2S_SDO_IN			GPIO_ReadInputDataBit(P2S_SDO_PORT,P2S_SDO_PIN)
#define P2S_LD_ON			{GPIO_SetBits(P2S_LD_PORT,P2S_LD_PIN);}
#define P2S_LD_OFF			{GPIO_ResetBits(P2S_LD_PORT,P2S_LD_PIN);}
#define P2S_CLK_ON			{GPIO_SetBits(P2S_CLK_PORT,P2S_CLK_PIN);}
#define P2S_CLK_OFF			{GPIO_ResetBits(P2S_CLK_PORT,P2S_CLK_PIN);}

void Driver_74hc165_Init(S74HC165TYPE* s74hc165)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO RCC */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = P2S_SDO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(P2S_SDO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = P2S_LD_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(P2S_LD_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = P2S_CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(P2S_CLK_PORT, &GPIO_InitStructure);
	
	P2S_LD_ON
	P2S_CLK_ON
}

uint32_t Driver_74hc165(S74HC165TYPE* s74hc165)
{
	uint32_t u32tmp;
	uint8_t t;
	uint8_t i;

	i = 0;
	u32tmp = 0x00;

	P2S_LD_ON
	P2S_CLK_ON
	P2S_LD_OFF
	P2S_LD_ON

	for(i = 0x00; i < 8*s74hc165->num; i ++)
	{	
		
		t = P2S_SDO_IN;
		if(t)
		{
			u32tmp |= 1;
		}

		if(i < 8*s74hc165->num - 1)
		{
			u32tmp = u32tmp << 1;
		}
		P2S_CLK_OFF
		P2S_CLK_ON
	}

	return u32tmp;
}


