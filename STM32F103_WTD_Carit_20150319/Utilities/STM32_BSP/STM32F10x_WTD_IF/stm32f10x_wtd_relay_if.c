#include <stdio.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* Add WTD drivers includes */
#include "stm32f10x_wtd_drivers.h"

/* Add WTD interfaces includes */
#include "stm32f10x_wtd_relay_if.h"

#if (WTD_RELAY_EN)

S74HC595TYPE m_stLed74595;
S74HC595TYPE m_stRly74595;
uint16_t u16_Relaypos;					   //继电器位置
uint16_t u16_Relaypos_old;				   //记录前一次继电器的位置

uint16_t u16_RelaySafypos;				   //继电器断电后，继电器需要保持的安全状态
uint16_t u16_RelaySafypos_old;

uint16_t u16_RelaySafy_timeout;		   //继电器断线不安全判断（MODBUS不通信时间）	
uint16_t u16_RelaySafy_timeout_old;

static uint32_t  u32_timeOld = 0;

uint16_t  u16_SafyEnable;
uint16_t  u16_SafyEnable_old;
uint16_t  u16_SafyOccur;                //是否进入安全模式的标志位

/*--------------- Private functions -----------------*/
/* Modbus Coil Interface */
static void Set_RelayState(int iIndex, int iState);
static int Get_RelayState(int iIndex);

/* Modbus Holding Registers Interface */
static void Set_RelayRegs(uint16_t usAddress, uint16_t usValue);
static uint16_t Get_RelayRegs(uint16_t usAddress);

void RelayCoilCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, uint8_t bWrite)
{
	int             iRegIndex = 0;
	int             iRegBit = 0;

	iRegIndex = ( int )( usAddress - RELAY_COIL_START );
	switch ( bWrite )
	{
	case pdFALSE:
		while( usNCoils > 0 )
		{
			iRegBit = Get_RelayState(iRegIndex);
			if(iRegIndex % 8 == 0)	// clear buffer
				pucRegBuffer[iRegIndex / 8] = 0;
			pucRegBuffer[iRegIndex / 8]	|= (iRegBit << (iRegIndex % 8));
			iRegIndex++;
			usNCoils--;
		}
		break;
	
	case pdTRUE:
		while( usNCoils > 0 )
		{
			iRegBit = ((pucRegBuffer[iRegIndex / 8] >> ((usNCoils - 1) % 8)) & 0x1);
			Set_RelayState(iRegIndex, iRegBit);
			iRegIndex++;
			usNCoils--;
		}
	}
}

void RelayHoldingRegCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, uint8_t bWrite)
{
	int             iRegIndex = 0;
	uint16_t          usRegValue = 0;	 

    iRegIndex = ( int )( usAddress - RELAY_HOLDINGREG_START );
    switch ( bWrite )
    {
    case pdFALSE:
        while( usNRegs > 0 )
        {
			usRegValue = Get_RelayRegs(usAddress);
            *pucRegBuffer++ = ( unsigned char )( usRegValue >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegValue & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
        break;

    case pdTRUE:
        while( usNRegs > 0 )
        {
			usRegValue = *pucRegBuffer++ << 8;
        	usRegValue |= *pucRegBuffer++;
			Set_RelayRegs(usAddress, usRegValue);
            iRegIndex++;
            usNRegs--;
        }
    }
}

uint16_t Get_SafyOccur(void)
{
	uint16_t ret;

	ret = u16_SafyOccur;	 //persecond 500;

	return ret;
}

void Set_MBRelayTime(void)
{
	u32_timeOld = xTaskGetTickCount();	 //persecond 500;
}

uint16_t Get_SafyEnable(void)
{
	uint16_t ret;

	ret = u16_SafyEnable;

	return ret;
}

void Set_SafyEnable(uint16_t val)
{
	u16_SafyEnable = val;
}

uint16_t Get_RelayPos(void)
{
	uint16_t ret;

	ret = u16_Relaypos;

	return ret;
}

void Set_RelayPos(uint16_t val)
{
	u16_Relaypos = val;
}

uint16_t Get_RelaySafypos(void)
{
	uint16_t ret;

	ret = u16_RelaySafypos;

	return ret;
}

void Set_RelaySafypos(uint16_t val)
{
 	u16_RelaySafypos = val;
}

uint16_t Get_RelaySafyTimeOut(void)
{
	uint16_t ret;

	ret = u16_RelaySafy_timeout;

	return ret;
}

void Set_RelaySafyTimeOut(uint16_t val)
{
 	u16_RelaySafy_timeout = val;
}

void Set_RelayState(int iIndex, int iState)
{
	if(iIndex < RELAY_COIL_NUM)
	{
		if(iState == 1)
			u16_Relaypos |= BIT(iIndex);
		else
			u16_Relaypos &= ~BIT(iIndex);
	}	
}

int Get_RelayState(int iIndex)
{
	int iState = 0;
	if(iIndex < RELAY_COIL_NUM)
	{
		iState = (u16_Relaypos & BIT(iIndex)) ? 1 : 0;
	}
	return iState;	
}

void Set_RelayRegs(uint16_t usAddress, uint16_t usValue)
{
	if(usAddress == RELAY_HOLDINGREG_START)
	{
		Set_SafyEnable(usValue);
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 1))
	{
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 2))
	{
		Set_RelaySafyTimeOut(usValue);
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 3))
	{
		Set_RelaySafypos(usValue);
	}
	else
	{
	}
}

uint16_t Get_RelayRegs(uint16_t usAddress)
{
	uint16_t ret = 0;

	if(usAddress == RELAY_HOLDINGREG_START)
	{
		ret = Get_SafyEnable();
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 1))
	{
		ret = Get_SafyOccur();
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 2))
	{
		ret = Get_RelaySafyTimeOut();
	}
	else if(usAddress == (RELAY_HOLDINGREG_START + 3))
	{
		ret = Get_RelaySafypos();
	}
	else
	{	ret = 0;	}

	return ret;
}

void Set_DOUT1(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT1_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT1_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT2(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT2_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT2_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT3(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT3_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT3_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT4(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT4_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT4_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT5(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT5_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT5_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT6(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT6_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT6_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT7(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT7_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT7_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT8(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT8_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT8_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT9(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT9_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT9_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT10(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT10_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT10_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT11(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT11_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT11_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT12(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT12_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT12_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT13(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT13_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT13_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT14(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT14_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT14_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT15(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT15_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT15_POS;
	}
	else
	{
		;
	}
}

void Set_DOUT16(uint8_t val)
{
	if(val == 0x01)
	{
		u16_Relaypos |= DOUT16_POS;
	}
	else if(val == 0x00)
	{
		u16_Relaypos &= ~DOUT16_POS;
	}
	else
	{
		;
	}
}

uint8_t Get_DOUT1(void)
{
	if(u16_Relaypos & DOUT1_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT2(void)
{
	if(u16_Relaypos & DOUT2_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT3(void)
{
	if(u16_Relaypos & DOUT3_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT4(void)
{
	if(u16_Relaypos & DOUT4_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT5(void)
{
	if(u16_Relaypos & DOUT5_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT6(void)
{
	if(u16_Relaypos & DOUT6_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT7(void)
{
	if(u16_Relaypos & DOUT7_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT8(void)
{
	if(u16_Relaypos & DOUT8_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT9(void)
{
	if(u16_Relaypos & DOUT9_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT10(void)
{
	if(u16_Relaypos & DOUT10_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT11(void)
{
	if(u16_Relaypos & DOUT11_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT12(void)
{
	if(u16_Relaypos & DOUT12_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT13(void)
{
	if(u16_Relaypos & DOUT13_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT14(void)
{
	if(u16_Relaypos & DOUT14_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT15(void)
{
	if(u16_Relaypos & DOUT15_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

uint8_t Get_DOUT16(void)
{
	if(u16_Relaypos & DOUT16_POS)
	{
		return 0x01;
	}
	else
	{
		return 0x00;
	}
}

void if_relay_init(void)
{
	uint8_t t;

	m_stLed74595.RCL_GPIO_Pin = GPIO_Pin_8;
	m_stLed74595.SCL_GPIO_Pin = GPIO_Pin_3;
	m_stLed74595.SDA_GPIO_Pin = GPIO_Pin_9;

	m_stLed74595.RCL_GPIOx = GPIOA;
	m_stLed74595.SCL_GPIOx = GPIOB;
	m_stLed74595.SDA_GPIOx = GPIOC;

	m_stRly74595.RCL_GPIO_Pin = GPIO_Pin_8;
	m_stRly74595.SCL_GPIO_Pin = GPIO_Pin_3;
	m_stRly74595.SDA_GPIO_Pin = GPIO_Pin_9;

	m_stRly74595.RCL_GPIOx = GPIOA;
	m_stRly74595.SCL_GPIOx = GPIOB;
	m_stRly74595.SDA_GPIOx = GPIOC;

	Driver_74hc595_Init(&m_stLed74595);
	Driver_74hc595_Init(&m_stRly74595);

#if WTD578C
	m_74165.SDO_GPIO_PIN = GPIO_Pin_8;
	m_74165.CLK_GPIO_Pin = GPIO_Pin_7;
	m_74165.LD_GPIO_Pin = GPIO_Pin_6;

	m_74165.SDO_GPIOx = GPIOB;
	m_74165.CLK_GPIOx = GPIOC;
	m_74165.LD_GPIOx = GPIOC;

	m_74165.num = 1;     //1路

	Driver_74hc165_Init(&m_74165);

	u16_Inputpos = 0x0000;
#endif

	u16_Relaypos = 0;
	u16_Relaypos_old = 0;
	E2P_ReadBKUP(E2P_SAFYPOSH_ADDR,&t);
	u16_RelaySafypos = t;
	u16_RelaySafypos = u16_RelaySafypos << 8;
	E2P_ReadBKUP(E2P_SAFYPOSL_ADDR,&t);
	u16_RelaySafypos += t;
#if WTD466C
	u16_RelaySafypos &= 0x007F;	//保留7位有效数字
#endif
	u16_RelaySafypos_old = u16_RelaySafypos;

	E2P_ReadBKUP(E2P_TIMEOUTH_ADDR,&t);
	u16_RelaySafy_timeout = t;
	u16_RelaySafy_timeout = u16_RelaySafy_timeout << 8;
	E2P_ReadBKUP(E2P_TIMEOUTL_ADDR,&t);
	u16_RelaySafy_timeout += t;
	u16_RelaySafy_timeout_old = u16_RelaySafy_timeout;

	E2P_ReadBKUP(E2P_SAFYENABLE_ADDR,&t);
	u16_SafyEnable = t;
	u16_SafyEnable_old = u16_SafyEnable;

	u16_SafyOccur = 0;

	u32_timeOld = xTaskGetTickCount();

	Set_DOUT7(1);

	/* Initialize output status */
	Driver_74hc595_Ctrl(&m_stLed74595,u16_Relaypos);
	Driver_74hc595_Ctrl(&m_stRly74595,u16_Relaypos);
}

void if_relay_Proc(void)
{
	uint32_t timeNow;
	uint8_t t;

	if(u16_Relaypos_old == u16_Relaypos)
	{//继电器无变化则无需输出

	}
	else
	{
		Driver_74hc595_Ctrl(&m_stLed74595,u16_Relaypos);
		Driver_74hc595_Ctrl(&m_stRly74595,u16_Relaypos);
		u16_Relaypos_old = u16_Relaypos;
	}

	if(u16_RelaySafypos_old == u16_RelaySafypos)
	{

	}
	else
	{//写入eeprom
		t = (u16_RelaySafypos &0xff);
		E2P_WriteBKUP(E2P_SAFYPOSL_ADDR,t);
		t = (u16_RelaySafypos >> 8) & 0xff;
		E2P_WriteBKUP(E2P_SAFYPOSH_ADDR,t);
		u16_RelaySafypos_old = u16_RelaySafypos;
	}

	if(u16_RelaySafy_timeout_old == u16_RelaySafy_timeout)
	{

	}
	else
	{
		t = (u16_RelaySafy_timeout &0xff);
		E2P_WriteBKUP(E2P_TIMEOUTL_ADDR,t);
		t = (u16_RelaySafy_timeout >> 8) & 0xff;
		E2P_WriteBKUP(E2P_TIMEOUTH_ADDR,t);
		u16_RelaySafy_timeout_old = u16_RelaySafy_timeout;
	}

	if(u16_SafyEnable_old == u16_SafyEnable)
	{

	}
	else
	{	
		t =u16_SafyEnable;
		E2P_WriteBKUP(E2P_SAFYENABLE_ADDR,t);
		u16_SafyEnable_old = u16_SafyEnable;
	}

	if(u16_SafyEnable == 1)
	{
		timeNow = xTaskGetTickCount();
		if((timeNow - u32_timeOld) > (u16_RelaySafy_timeout * 100)) //0.1s的单位转换->systick
		{//超时，需要强制继电器模块进入安全模式
			Driver_74hc595_Ctrl(&m_stLed74595,u16_RelaySafypos);
			Driver_74hc595_Ctrl(&m_stRly74595,u16_Relaypos);
			u16_Relaypos = u16_RelaySafypos & 0x00FF;
			u16_SafyOccur = 1;
		}
		else
		{
			u16_SafyOccur = 0;
		}
	}

#if WTD578C
	u16_Inputpos = Driver_74hc595_Ctrl(&m_74165);
	u16_Inputpos = ~u16_Inputpos;
	u16_Relaypos &= ~(0xFF00);
	u16_Relaypos |= (u16_Inputpos << 8);
#endif

}

#endif
