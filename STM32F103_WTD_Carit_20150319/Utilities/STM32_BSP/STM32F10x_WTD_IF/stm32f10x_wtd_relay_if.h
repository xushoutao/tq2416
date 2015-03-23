#ifndef __STM32F10X_WTD_RELAY_IF_H__
#define __STM32F10X_WTD_RELAY_IF_H__

#include "FreeRTOS.h"
/* Add WTD config includes */
#include "wtdconf.h"

#define BIT(x)		(1 << x)

#if (WTD_RELAY_EN)

/* Define relay module modbus coil addr */
#define RELAY_COIL_START		(0x11)
#define RELAY_COIL_NUM			(7)
void RelayCoilCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, uint8_t bWrite);

/* Define relay module modbus hoding reg addr */
#define RELAY_HOLDINGREG_START         (0x0105)
#define RELAY_HOLDINGREG_NUM           (4)
void RelayHoldingRegCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, uint8_t bWrite);

#define DOUT1_POS		BIT(0)
#define DOUT2_POS		BIT(1)
#define DOUT3_POS		BIT(2)
#define DOUT4_POS		BIT(3)
#define DOUT5_POS		BIT(4)
#define DOUT6_POS		BIT(5)
#define DOUT7_POS		BIT(6)
#define DOUT8_POS		BIT(7)
#define DOUT9_POS		BIT(8)
#define DOUT10_POS		BIT(9)
#define DOUT11_POS		BIT(10)
#define DOUT12_POS		BIT(11)
#define DOUT13_POS		BIT(12)
#define DOUT14_POS		BIT(13)
#define DOUT15_POS		BIT(14)
#define DOUT16_POS		BIT(15)

void Set_SafyEnable(uint16_t val);
void Set_RelayPos(uint16_t val);
void Set_DOUT1(uint8_t val);
void Set_DOUT2(uint8_t val);
void Set_DOUT3(uint8_t val);
void Set_DOUT4(uint8_t val);
void Set_DOUT5(uint8_t val);
void Set_DOUT6(uint8_t val);
void Set_DOUT7(uint8_t val);
void Set_DOUT8(uint8_t val);
void Set_DOUT9(uint8_t val);
void Set_DOUT10(uint8_t val);
void Set_DOUT11(uint8_t val);
void Set_DOUT12(uint8_t val);
void Set_DOUT13(uint8_t val);
void Set_DOUT14(uint8_t val);
void Set_DOUT15(uint8_t val);
void Set_DOUT16(uint8_t val);
void Set_RelaySafypos(uint16_t val);
void Set_RelaySafyTimeOut(uint16_t val);

uint16_t Get_SafyOccur(void);
uint16_t Get_SafyEnable(void);
uint16_t Get_RelayPos(void);
uint8_t Get_DOUT1(void);
uint8_t Get_DOUT2(void);		
uint8_t Get_DOUT3(void);
uint8_t Get_DOUT4(void);
uint8_t Get_DOUT5(void);
uint8_t Get_DOUT6(void);
uint8_t Get_DOUT7(void);
uint8_t Get_DOUT8(void);
uint8_t Get_DOUT9(void);
uint8_t Get_DOUT10(void);
uint8_t Get_DOUT11(void);
uint8_t Get_DOUT12(void);
uint8_t Get_DOUT13(void);
uint8_t Get_DOUT14(void);
uint8_t Get_DOUT15(void);
uint8_t Get_DOUT16(void);
uint16_t Get_RelaySafypos(void);
uint16_t Get_RelaySafyTimeOut(void);

void if_relay_init(void);
void if_relay_Proc(void);
void Set_MBRelayTime(void);

#endif /* #if WTD566C */

#endif /* __STM32F10X_WTD_RELAY_IF_H */
