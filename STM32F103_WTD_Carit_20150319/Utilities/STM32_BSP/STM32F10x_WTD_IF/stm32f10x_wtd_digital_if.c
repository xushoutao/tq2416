#include <stdio.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* Add WTD drivers includes */
#include "stm32f10x_wtd_drivers.h"

/* Add WTD interfaces includes */
#include "stm32f10x_wtd_digital_if.h"

#if (WTD_DIGITAL_EN)
/* Private Parameters */
/* Declare digital output/input serial to paralle object */
static S74HC595TYPE l_stDig74595;

static uint16_t l_usCoilState;			//线圈状态按位表示
static uint16_t l_usCoilState_old;		//记录前一次继电器的位置
										
static uint16_t l_usSafyCoilState;		//断电后，线圈需要保持的安全状态
static uint16_t l_usSafyCoilState_old;

static uint16_t l_usSafyEnable;
static uint16_t l_usSafyEnable_old;
static uint16_t l_usSafyTimeout;			//线圈断线不安全判断（MODBUS不通信时间）	
static uint16_t l_usSafyTimeout_old;
static uint16_t l_usSafyOccur;			//是否进入安全模式的标志位

static uint32_t l_uiTimeOld = 0;			//记录旧的时间

/*--------------- Private functions -----------------*/
/* Modbus Coil Interface */
static void Set_DigitalState(int iIndex, int iState);
static int Get_DigitalState(int iIndex);

/* Modbus Holding Registers Interface */
static void Set_DigitalRegs(uint16_t usAddress, uint16_t usValue);
static uint16_t Get_DigitalRegs(uint16_t usAddress);

/* Functions */
uint16_t Digital_GetSafyOccur(void);
uint16_t Digital_GetSafyEnable(void);
void Digital_SetSafyEnable(uint16_t val);
uint16_t Digital_GetSafyCoilState(void);
void Digital_SetSafyCoilState(uint16_t val);
uint16_t Digital_GetSafyTimeOut(void);
void Digital_SetSafyTimeOut(uint16_t val);

/*------------------------------------------------------------------------------------------------------------*/
void DigitalCoilCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils, uint8_t bWrite)
{
	int             iRegIndex = 0;
	int             iRegBit = 0;

	iRegIndex = ( int )( usAddress - DIGITAL_COIL_START );
	switch ( bWrite )
	{
	case pdFALSE:
		while( usNCoils > 0 )
		{
			iRegBit = Get_DigitalState(iRegIndex);
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
			Set_DigitalState(iRegIndex, iRegBit);
			iRegIndex++;
			usNCoils--;
		}
	}
}

void DigitalHoldingRegCB(uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs, uint8_t bWrite)
{
	int             iRegIndex = 0;
	uint16_t          usRegValue = 0;	 

    iRegIndex = ( int )( usAddress - DIGITAL_HOLDINGREG_START );
    switch ( bWrite )
    {
    case pdFALSE:
        while( usNRegs > 0 )
        {
			usRegValue = Get_DigitalRegs(usAddress);
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
			Set_DigitalRegs(usAddress, usRegValue);
            iRegIndex++;
            usNRegs--;
        }
    }
}

void Set_DigitalState(int iIndex, int iState)
{
	if(iIndex < DIGITAL_COIL_NUM)
	{
		if(iState == 1)
			l_usCoilState |= BIT(iIndex);
		else
			l_usCoilState &= ~BIT(iIndex);
	}	
}

int Get_DigitalState(int iIndex)
{
	int iState = 0;
	if(iIndex < DIGITAL_COIL_NUM)
	{
		iState = (l_usCoilState & BIT(iIndex)) ? 1 : 0;
	}
	return iState;	
}

void Set_DigitalRegs(uint16_t usAddress, uint16_t usValue)
{
	if(usAddress == DIGITAL_HOLDINGREG_START)
	{
		Digital_SetSafyEnable(usValue);
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 1))
	{
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 2))
	{
		Digital_SetSafyTimeOut(usValue);
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 3))
	{
		Digital_SetSafyCoilState(usValue);
	}
	else
	{
	}
}

uint16_t Get_DigitalRegs(uint16_t usAddress)
{
	uint16_t ret = 0;

	if(usAddress == DIGITAL_HOLDINGREG_START)
	{
		ret = Digital_GetSafyEnable();
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 1))
	{
		ret = Digital_GetSafyOccur();
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 2))
	{
		ret = Digital_GetSafyTimeOut();
	}
	else if(usAddress == (DIGITAL_HOLDINGREG_START + 3))
	{
		ret = Digital_GetSafyCoilState();
	}
	else
	{	ret = 0;	}

	return ret;
}

uint16_t Digital_GetSafyOccur(void)
{
	uint16_t ret;

	ret = l_usSafyOccur;	 //persecond 500;

	return ret;
}

uint16_t Digital_GetSafyEnable(void)
{
	uint16_t ret;

	ret = l_usSafyEnable;

	return ret;
}

void Digital_SetSafyEnable(uint16_t val)
{
	l_usSafyEnable = val;
}

uint16_t Digital_GetSafyCoilState(void)
{
	uint16_t ret;

	ret = l_usSafyCoilState;

	return ret;
}

void Digital_SetSafyCoilState(uint16_t val)
{
 	l_usSafyCoilState = val;
}

uint16_t Digital_GetSafyTimeOut(void)
{
	uint16_t ret;

	ret = l_usSafyTimeout;

	return ret;
}

void Digital_SetSafyTimeOut(uint16_t val)
{
 	l_usSafyTimeout = val;
}

void if_Digital_init(void)
{
	uint8_t ucTmp;

	/* Led signal init */
	Driver_LedInit();

	l_stDig74595.RCL_GPIO_Pin = GPIO_Pin_8;
	l_stDig74595.SCL_GPIO_Pin = GPIO_Pin_3;
	l_stDig74595.SDA_GPIO_Pin = GPIO_Pin_9;

	l_stDig74595.RCL_GPIOx = GPIOA;
	l_stDig74595.SCL_GPIOx = GPIOB;
	l_stDig74595.SDA_GPIOx = GPIOC;

	Driver_74hc595_Init(&l_stDig74595);

#if WTD578C
#endif

	l_usCoilState = 0;
	l_usCoilState_old = 0;
	
	/* Set safety coil state */
	E2P_ReadBKUP(E2P_SAFYPOSH_ADDR,&ucTmp);
	l_usSafyCoilState = ucTmp;
	l_usSafyCoilState <<= 8;
	E2P_ReadBKUP(E2P_SAFYPOSL_ADDR,&ucTmp);
	l_usSafyCoilState += ucTmp;
	l_usSafyCoilState_old = l_usSafyCoilState;

	E2P_ReadBKUP(E2P_TIMEOUTH_ADDR,&ucTmp);
	l_usSafyTimeout = ucTmp;
	l_usSafyTimeout = l_usSafyTimeout << 8;
	E2P_ReadBKUP(E2P_TIMEOUTL_ADDR,&ucTmp);
	l_usSafyTimeout += ucTmp;
	l_usSafyTimeout_old = l_usSafyTimeout;

	E2P_ReadBKUP(E2P_SAFYENABLE_ADDR,&ucTmp);
	l_usSafyEnable = ucTmp;
	l_usSafyEnable_old = l_usSafyEnable;

	l_usSafyOccur = 0;

	l_uiTimeOld = xTaskGetTickCount();

//	Set_DOUT7(1);
	Set_DigitalState(7, 1);

	/* Initialize output status */
	Driver_LedSToPCtrl(l_usCoilState);
	Driver_74hc595_Ctrl(&l_stDig74595, l_usCoilState);
}

void if_Digital_Proc(void)
{
	uint32_t uiTimeNow;
	uint8_t ucTmp;

	if(l_usCoilState_old == l_usCoilState)
	{//线圈无变化则无需输出

	}
	else
	{
		Driver_LedSToPCtrl(l_usCoilState);
		Driver_74hc595_Ctrl(&l_stDig74595, l_usCoilState);
		l_usCoilState_old = l_usCoilState;
	}

	if(l_usSafyCoilState_old == l_usSafyCoilState)
	{

	}
	else
	{//写入eeprom
		ucTmp = (l_usSafyCoilState & 0xff);
		E2P_WriteBKUP(E2P_SAFYPOSL_ADDR, ucTmp);
		ucTmp = (l_usSafyCoilState >> 8) & 0xff;
		E2P_WriteBKUP(E2P_SAFYPOSH_ADDR, ucTmp);
		l_usSafyCoilState_old = l_usSafyCoilState;
	}

	if(l_usSafyTimeout_old == l_usSafyTimeout)
	{

	}
	else
	{
		ucTmp = (l_usSafyTimeout & 0xff);
		E2P_WriteBKUP(E2P_TIMEOUTL_ADDR, ucTmp);
		ucTmp = (l_usSafyTimeout >> 8) & 0xff;
		E2P_WriteBKUP(E2P_TIMEOUTH_ADDR, ucTmp);
		l_usSafyTimeout_old = l_usSafyTimeout;
	}

	if(l_usSafyEnable_old == l_usSafyEnable)
	{

	}
	else
	{	
		ucTmp = l_usSafyEnable;
		E2P_WriteBKUP(E2P_SAFYENABLE_ADDR, ucTmp);
		l_usSafyEnable_old = l_usSafyEnable;
	}

	if(l_usSafyEnable == 1)
	{
		uiTimeNow = xTaskGetTickCount();
		if((uiTimeNow - l_uiTimeOld) > (l_usSafyTimeout * 100)) //0.1s的单位转换->systick
		{//超时，需要强制继电器模块进入安全模式
			Driver_LedSToPCtrl(l_usSafyCoilState);
			Driver_74hc595_Ctrl(&l_stDig74595,l_usSafyCoilState);
			l_usCoilState = l_usSafyCoilState & 0x00FF;
			l_usSafyOccur = 1;
		}
		else
		{
			l_usSafyOccur = 0;
		}
	}
}

#endif /* WTD_DIGITAL_EN */
