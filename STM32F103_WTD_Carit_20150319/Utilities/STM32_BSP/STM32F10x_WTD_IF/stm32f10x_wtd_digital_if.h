#ifndef __STM32F10X_WTD_DIGITAL_IF_H__
#define __STM32F10X_WTD_DIGITAL_IF_H__

#include "FreeRTOS.h"
/* Add WTD config includes */
#include "wtdconf.h"

#define BIT(x)		(1 << x)

#if (WTD_DIGITAL_EN)

/* Define digital module modbus coil addr */
#define DIGITAL_COIL_START			(0x11)
#define DIGITAL_COIL_NUM			(16)
void DigitalCoilCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, uint8_t bWrite);

/* Define digital module modbus hoding reg addr */
#define DIGITAL_HOLDINGREG_START	(0x0105)
#define DIGITAL_HOLDINGREG_NUM		(4)
void DigitalHoldingRegCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, uint8_t bWrite);

#endif /* WTD_DIGITAL_EN */

#endif /* __STM32F10X_WTD_DIGITAL_IF_H__ */
