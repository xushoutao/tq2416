/********************************************************************************************************
**包含必要的头文件
*********************************************************************************************************/
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

#define IRQnToEXPn(IRQn)		(IRQn + 16)
#if DRIVER_INT_EN > 0

#ifdef STM32F10X_MD
#define NUM_INTERRUPTS 64
#endif

//static void __align(512) (* l_pfnVectors[NUM_INTERRUPTS])(void);
static void (* l_pfnVectors[NUM_INTERRUPTS])(void) __attribute__((at(0x20000000)));

extern void *__Vectors[];
//extern void *__Vectors_End[];
extern uint32_t *__Vectors_Size[];

static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

void Driver_IntRelocate(void)
{
	__disable_irq();

	//printf("address: 0x%08x", l_pfnVectors);

	memcpy(l_pfnVectors, (void *)__Vectors, (uint32_t)__Vectors_Size);

	/* Set the Vector Table base address at 0x20000000 */
	NVIC_SetVectorTable( NVIC_VectTab_RAM, (uint32_t)l_pfnVectors );

	__enable_irq();
}

void
Driver_IntRegister(unsigned long ulInterrupt, void (*pfnHandler)(void))
{
	unsigned long *ulNvicTbl;
	
	//
	// Check the arguments.
	//
	//ASSERT(ulInterrupt < NUM_INTERRUPTS);
	
	ulNvicTbl = (unsigned long *)SCB->VTOR;
	ulNvicTbl[IRQnToEXPn(ulInterrupt)]= (unsigned long)pfnHandler;
}

void
Driver_IntUnregister(unsigned long ulInterrupt)
{
	unsigned long *ulNvicTbl;
	
	//
	// Check the arguments.
	//
	//ASSERT(ulInterrupt < NUM_INTERRUPTS);
	
	ulNvicTbl = (unsigned long *)SCB->VTOR;
	ulNvicTbl[IRQnToEXPn(ulInterrupt)]= (unsigned long)IntDefaultHandler;
}

#endif
