#ifndef __STM32F2X7_WTD_DRIVERS_H__
#define __STM32F2X7_WTD_DRIVERS_H__

#define DRIVER_USART_EN        1
#define DRIVER_EEPROM_EN       1
#define DRIVER_SPI_EN          1
#define DRIVER_74HC165_EN      1
#define DRIVER_74HC595_EN      1
#define DRIVER_LED_EN          1
#define DRIVER_INT_EN          1

#if DRIVER_USART_EN
#include "stm32f10x_wtd_usart.h"
#endif

#if DRIVER_EEPROM_EN
#include "stm32f10x_wtd_eeprom.h"
#endif

#if DRIVER_SPI_EN
#include "stm32f10x_wtd_spi.h"
#endif

#if DRIVER_74HC165_EN
#include "stm32f10x_wtd_74hc165.h"
#endif

#if DRIVER_74HC595_EN
#include "stm32f10x_wtd_74hc595.h"
#endif

#if DRIVER_LED_EN
#include "stm32f10x_wtd_led.h"
#endif

#if DRIVER_INT_EN
#include "stm32f10x_wtd_int.h"
#endif

#endif /* __STM32F2X7_WTD_DRIVERS_H__ */

