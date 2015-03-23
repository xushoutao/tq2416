#ifndef __STM32F10x_WTD_74HC595_H__
#define __STM32F10X_WTD_74HC595_H__

typedef struct  
{
	GPIO_TypeDef* SCL_GPIOx;     //GPIO ¶Ë¿ÚºÅ
	uint16_t SCL_GPIO_Pin;			//GPIO Òý½Å±àºÅ
	GPIO_TypeDef* RCL_GPIOx;     //GPIO ¶Ë¿ÚºÅ
	uint16_t RCL_GPIO_Pin;			//GPIO Òý½Å±àºÅ
	GPIO_TypeDef* SDA_GPIOx;     //GPIO ¶Ë¿ÚºÅ
	uint16_t SDA_GPIO_Pin;			//GPIO Òý½Å±àºÅ
}S74HC595TYPE;

void Driver_74hc595_Init(S74HC595TYPE* s74hc595);
void Driver_74hc595_Ctrl(S74HC595TYPE* s74hc595, uint16_t usState);

#endif /* __STM32F10X_WTD_74HC595_H__ */
