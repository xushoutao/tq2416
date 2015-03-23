#ifndef __STM32F10X_WTD_74HC165_H__
#define __STM32F10X_WTD_74HC165_H__

typedef struct  
{
	GPIO_TypeDef* SDO_GPIOx;     //GPIO 端口号
	uint16_t SDO_GPIO_PIN;			//GPIO 引脚编号
	GPIO_TypeDef* CLK_GPIOx;     //GPIO 端口号
	uint16_t CLK_GPIO_Pin;			//GPIO 引脚编号
	GPIO_TypeDef* LD_GPIOx;     //GPIO 端口号
	uint16_t LD_GPIO_Pin;			//GPIO 引脚编号
	u32 num;                              //设备路数
}S74HC165TYPE;

void Driver_74hc165_Init(S74HC165TYPE* s74hc165);
uint32_t Driver_74hc165(S74HC165TYPE* s74hc165);

#endif /* __STM32F10X_WTD_74HC165_H__ */
