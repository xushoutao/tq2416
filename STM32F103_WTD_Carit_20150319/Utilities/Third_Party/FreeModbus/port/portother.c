/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portother.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"

/* ----------------------- Variables ----------------------------------------*/
static BOOL     bIsWithinException = FALSE;
static SemaphoreHandle_t xMBSemaphore = NULL;
#if MB_MASTER_ENABLED > 0
static BOOL		bWaitEnable = FALSE;
static SemaphoreHandle_t xMBMasterWaitSem = NULL;
#endif

/* ----------------------- Start implementation -----------------------------*/

void
vMBPortSetWithinException( BOOL bInException )
{
    bIsWithinException = bInException;
}

BOOL
bMBPortIsWithinException( void )
{
    return bIsWithinException;
}

void
vMBPortEnterCritical( void )
{
    taskENTER_CRITICAL(  );
}

void
vMBPortExitCritical( void )
{
    taskEXIT_CRITICAL(  );
}

void
vMBPortMutexLock( void )
{
	if(xMBSemaphore == NULL)
		vSemaphoreCreateBinary(xMBSemaphore);
	if(xMBSemaphore != NULL)
		xSemaphoreTake(xMBSemaphore, portMAX_DELAY);
}

void
vMBPortMutexUnlock( void )
{
	if(xMBSemaphore != NULL)
		xSemaphoreGive(xMBSemaphore);
}

#if MB_MASTER_ENABLED > 0
BOOL
vMBPortWaitStart(ULONG ulBlockTime)
{
	if(xMBMasterWaitSem == NULL)
		xMBMasterWaitSem = xSemaphoreCreateBinary();
	if(xMBMasterWaitSem != NULL && bWaitEnable == FALSE)
	{
		bWaitEnable = TRUE;
		if(xSemaphoreTake(xMBMasterWaitSem, (TickType_t) ulBlockTime))
		{
			bWaitEnable = FALSE;
			return TRUE;
		}
		else
		{
			bWaitEnable = FALSE;
			return FALSE;
		}
	}

	return FALSE;
}

void
vMBPortWaitEnd()
{
	if(xMBMasterWaitSem != NULL && bWaitEnable == TRUE)
	{
		if( bMBPortIsWithinException(  ) )
		{
			xSemaphoreGive(xMBMasterWaitSem); //xSemaphoreGiveFromISR
		}
		else
		{
			xSemaphoreGive(xMBMasterWaitSem);
		}		
		bWaitEnable = FALSE;
	}
}
#endif

void
vMBPortClose( void )
{
    extern void     vMBPortSerialClose( void );
    extern void     vMBPortTimerClose( void );
    extern void     vMBPortEventClose( void );
    vMBPortSerialClose(  );
    vMBPortTimerClose(  );
    vMBPortEventClose(  );
}
