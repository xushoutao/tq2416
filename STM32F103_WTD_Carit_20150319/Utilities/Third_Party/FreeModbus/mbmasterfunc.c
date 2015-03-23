/* 
 * FreeModbus Function: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2014 Carit Zhu <carit.zhu@witium.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbmasterfunc.c,v 1.0 2014/09/18 17:43:56 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbfunc.h"

#if MB_MASTER_ENABLED > 0

/* ----------------------- Extern variables ---------------------------------*/
extern UCHAR			ucMBAddress;
extern UCHAR			*ucMBFrame;
extern USHORT			usLength;

/* ----------------------- Static variables ---------------------------------*/
static UCHAR * pucDataDest;
static USHORT * pusDataDest;

#if MB_FUNC_READ_COILS_ENABLED > 0
eMBErrorCode
eModbus_Read_Coil_Status( UINT8 slave, UINT16 start_addr, 
						UINT16 number, UINT8 *data_dest )
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncReadCoils(ucMBFrame, start_addr, number, &usLength);
	pucDataDest = data_dest;
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
eMBErrorCode 
eModbus_Read_Input_Status( UINT8 slave, UINT16 start_addr, 
						UINT16 number, UINT8 *data_dest )
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncReadDiscreteInputs(ucMBFrame, start_addr, number, &usLength);
	pucDataDest = data_dest;
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

#if MB_FUNC_READ_HOLDING_ENABLED > 0
eMBErrorCode 
eModbus_Read_Holding_Register( UINT8 slave, UINT16 start_addr, 
							UINT16 number, UINT16 *data_dest )
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncReadHoldingRegister(ucMBFrame, start_addr, number, &usLength);
	pusDataDest = data_dest;
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

#if MB_FUNC_READ_INPUT_ENABLED > 0
eMBErrorCode 
eModbus_Read_Input_Register(UINT8 slave, UINT16 start_addr, 
							UINT16 number, UINT16 *data_dest )
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncReadInputRegister(ucMBFrame, start_addr, number, &usLength);
	pusDataDest = data_dest;
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

#if MB_FUNC_WRITE_COIL_ENABLED > 0
eMBErrorCode 
eModbus_Write_Single_Coil(UINT8 slave, UINT16 coil_addr, UINT16 state)
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncWriteCoil( ucMBFrame, coil_addr, state, &usLength);
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
eMBErrorCode 
eModbus_Write_Multiple_Coils(UINT8 slave, UINT16 start_addr, 
							UINT16 number, const UINT8 *data_src)
{
	eMBErrorCode eStatus = MB_ENOERR;
	vMBPortMutexLock();
	ucMBAddress = slave;
	vMBFrameGetBuffer(&ucMBFrame);
	vMBFuncWriteMultipleCoils(ucMBFrame, start_addr, number, data_src, &usLength);
	xMBPortEventPost(EV_SEND_FRAME);
	eStatus = eMBStartPolling();
	vMBPortMutexUnlock();
	return eStatus;
}
#endif

//注：加入对ucByteCount长度的判断(ucByteCount != usLength - 2)，
//不能超过usLength - 2长度。否则视为错误。
eMBException eMBRecvCBFuncReadDiscrete( UCHAR * pucFrame, USHORT * pusLength )
{
	eMBException eState = MB_EX_NONE;
	UCHAR ucByteCount;
	UCHAR i;

	if (*pusLength > MB_PDU_SIZE_MAX)
		return MB_EX_ILLEGAL_DATA_VALUE;

	ucByteCount = pucFrame[ MB_PDU_DATA_OFF ];					// 寄存器个数
	if ( ucByteCount == 0 || ucByteCount != usLength - 2 || pucDataDest == NULL )
	{
		eState = MB_EX_ILLEGAL_DATA_VALUE;
		return eState;
	}
	for( i = 0; i < ucByteCount; i++ )
	{
		pucDataDest[ i ] = pucFrame[ MB_PDU_DATA_OFF + 1 + i ];
	}
	pucDataDest = NULL;
	return eState;
}

eMBException eMBRecvCBFuncReadReg( UCHAR * pucFrame, USHORT * pusLength )
{
	eMBException eState = MB_EX_NONE;
	UCHAR ucByteCount;
	UCHAR * pucDataPtr;
	UCHAR i;

	if (*pusLength > MB_PDU_SIZE_MAX)
		return MB_EX_ILLEGAL_DATA_VALUE;

	ucByteCount = pucFrame[ MB_PDU_DATA_OFF ];	// 寄存器个数
	if ( ucByteCount == 0 || ucByteCount != usLength - 2 || pusDataDest == NULL )
	{
		eState = MB_EX_ILLEGAL_DATA_VALUE;
		return eState;
	}

	pucDataPtr = &pucFrame[ MB_PDU_DATA_OFF + 1 ];		// 数据或起始指针
	for( i = 0; i < ( ucByteCount / 2 ); i++)
	{
		*( pusDataDest + i ) = ((pucDataPtr[ i * 2] << 8) | pucDataPtr[ i * 2 + 1 ]);
	}
	pusDataDest = NULL;
	return eState;
}

eMBException eMBRecvCBFuncDefault( UCHAR * pucFrame, USHORT * pusLength )
{
	return MB_EX_NONE;
}

eMBException eMBRecvCBFuncException( UCHAR * pucFrame, USHORT * pusLength )
{
	eMBException eState = MB_EX_NONE;

	if (*pusLength > MB_PDU_SIZE_MAX)
		return MB_EX_ILLEGAL_DATA_VALUE;

	eState = (eMBException)(pucFrame[ MB_PDU_DATA_OFF ]);
	return eState;
}

#endif
