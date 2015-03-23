/****************************************Copyright (c)**************************************************
**                             
**              company:  Witium Intelligent System Co., Ltd.
**				      上海辉度智能系统有限公司   
**              WEB:      www.witium.com                  
**              TEL:      086-21-37774020
**			    FAX:      086-21-37774010
**			    Writer:   Carit.Zhu    
**              mobile:   18916777010
**              Email:    Carit.Zhu@witium.com
**
**--------------File Info-------------------------------------------------------------------------------
** File Name:               Driver_Spi.c
** Last modified Date:      2014.12.5
** Last Version:            v1.0
** Description:             STM32 SPI 驱动源文件
** 
**------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:          
** Version:                 
** Description:             
**
**------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:          
** Version:                 
** Description:             
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

/********************************************************************************************************
**包含必要的头文件
*********************************************************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

/********************************************************************************************************
** 为驱动程序的全局变量赋初值
*********************************************************************************************************/
#if DRIVER_SPI_EN > 0
#define BUFFER_SIZE       32
uint8_t SPI_MASTER_Buffer_Tx[BUFFER_SIZE] = {0};
uint8_t SPI_MASTER_Buffer_Rx[BUFFER_SIZE] = {0};
#endif

#if DRIVER_SPI_EN > 0

#define  SPI_CHANNEL_1                            SPI1
#define  SPI_PORT_1						          GPIOA
#define  SPI_CLK_1                                RCC_APB2Periph_SPI1
#define  SPI_PORT_CLK_1                           RCC_APB2Periph_GPIOA
#define  SPI_PORT_SPEED_1                         GPIO_Speed_50MHz
#define  SPI_PORT_MODE_1                          GPIO_Mode_AF_PP
#define  SPI_NSS_PIN_1                            GPIO_Pin_4
#define  SPI_SCK_PIN_1                            GPIO_Pin_5
#define  SPI_MISO_PIN_1                           GPIO_Pin_6
#define  SPI_MOSI_PIN_1                           GPIO_Pin_7
#define  SPI_TRANS_MODE_1                         SPI_Mode_Master
#define  SPI_CPOL_MODE_1                          SPI_CPOL_Low
#define  SPI_CPHA_MODE_1						  SPI_CPHA_1Edge
#define  SPI_DR_Base_1                            (uint32_t)&SPI1->DR //0x4000380C

#define  SPI_CHANNEL_2                            SPI2
#define  SPI_PORT_2                               GPIOB
#define  SPI_CLK_2								  RCC_APB1Periph_SPI2
#define  SPI_PORT_CLK_2                           RCC_APB2Periph_GPIOB
#define  SPI_PORT_SPEED_2                         GPIO_Speed_50MHz
#define  SPI_PORT_MODE_2                          GPIO_Mode_AF_PP
#define  SPI_NSS_PIN_2                            GPIO_Pin_12
#define  SPI_SCK_PIN_2                            GPIO_Pin_13
#define  SPI_MISO_PIN_2                           GPIO_Pin_14
#define  SPI_MOSI_PIN_2                           GPIO_Pin_15
#define  SPI_TRANS_MODE_2                         SPI_Mode_Master
#define  SPI_CPOL_MODE_2						  SPI_CPOL_Low
#define  SPI_CPHA_MODE_2						  SPI_CPHA_2Edge
#define  SPI_DR_Base_2                            (uint32_t)&SPI2->DR //0x4001380C

#define  SPI_MASTER								  SPI_CHANNEL_1
#define  SPI_MASTER_DMA                           DMA1
#define  SPI_MASTER_DMA_CLK                       RCC_AHBPeriph_DMA1  
#define  SPI_MASTER_Rx_DMA_Channel                DMA1_Channel4
#define  SPI_MASTER_Rx_DMA_FLAG                   DMA1_FLAG_TC4
#define  SPI_MASTER_Tx_DMA_Channel                DMA1_Channel5
#define  SPI_MASTER_Tx_DMA_FLAG                   DMA1_FLAG_TC5
#define  SPI_MASTER_DR_Base                       SPI_DR_Base_1
#define  SPI_MASTER_DUMMY_WRITE					  (0xFF)

typedef enum _SPI_TRANS_MODE
{
	SPI_Mode_Tx,
	SPI_Mode_Rx,
}SPI_TransMode;

#endif

#if DRIVER_SPI_EN > 0
  static void Spi_GPIOInit(void);

/** @defgroup STM32_SPI_Macros
  * @{
  */
/**
  * @brief  Select SPI: Chip Select pin low
  */
//  static void SPI_CSEnable(SPI_TransMode mode);
  #define SPI_CSEnable() GPIO_ResetBits(SPI_PORT_1, SPI_NSS_PIN_1)
/**
  * @brief  Deselect SPI: Chip Select pin high
  */
//  static void SPI_CSDisable(void);
  #define SPI_CSDisable() GPIO_SetBits(SPI_PORT_1, SPI_NSS_PIN_1)   
#endif

/****************************
 * 为驱动程序的内部功能函数
 ****************************/

#if DRIVER_SPI_EN > 0
void Spi_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Init GPIO periph clock */
	RCC_APB2PeriphClockCmd(SPI_PORT_CLK_1, ENABLE);
	RCC_APB2PeriphClockCmd(SPI_PORT_CLK_2, ENABLE);

	/* Enable SPI1 Pins Software Remapping */
	//由于用的还是PA4~7所以不需要重映射

	/* Configure SPI_NSS SPI_SCK SPI_MOSI as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN_1 | SPI_MOSI_PIN_1;
	GPIO_InitStructure.GPIO_Speed = SPI_PORT_SPEED_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_PORT_1, &GPIO_InitStructure);

	/* Configure SPI_MISO as alternate function in-floating */
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_PORT_1, &GPIO_InitStructure);

	/*!< Configure CS_PIN pin as output */
	GPIO_InitStructure.GPIO_Pin = SPI_NSS_PIN_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_PORT_1, &GPIO_InitStructure);
}
#endif


/*********************************************************************************************************
** Function name:           
** Descriptions:            
** input parameters:        
**                                    
** output parameters:       无      
** Returned value:          无	 
** Created By:              Carit.Zhu
** Created date:            2008.12.14
**--------------------------------------------------------------------------------------------------------
** Modified by:            
** Modified date:           
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#if DRIVER_SPI_EN > 0
int Driver_Spi_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	/* SPI GPIO Init */
	Spi_GPIOInit();

	/* SPI Periph clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* SPI_MASTER configuration ------------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	//SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_TRANS_MODE_1;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_MODE_1;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_MODE_1;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_MASTER, &SPI_InitStructure);
	
	/* Enable SPI1 NSS output for master mode */
	//SPI_SSOutputCmd(SPI_MASTER, ENABLE);
	
	/* Enable SPI_MASTER DMA */
	//Driver_Spi_DMAInit();
	
	/* Enable SPI */
	SPI_Cmd(SPI_MASTER, ENABLE);

	return 1;
}

Fd_t Driver_Spi_Open(char *ifName, unsigned long flags)
{
	Fd_t fd = 0;
	if(Driver_Spi_Init() == 1)
		fd = 1;

	return fd;
}

#endif

#if DRIVER_SPI_EN > 0
int Driver_Spi_DeInit(void)
{
	/* Disable SPI1 NSS output for master mode */
	SPI_SSOutputCmd(SPI_MASTER, DISABLE);

	/* Disable SPI */
	SPI_Cmd(SPI_MASTER, DISABLE);

	return 1;
}

int Driver_Spi_Close(Fd_t fd)
{
	Driver_Spi_DeInit();

	return 0;
}

//void SPI_CSEnable(SPI_TransMode mode)
//{
//	SPI_InitTypeDef SPI_InitStructure;
//	SPI_StructInit(&SPI_InitStructure);
//
//	/* Chip Select Pin Low for Enable */
//	GPIO_ResetBits(SPI_PORT_1, SPI_NSS_PIN_1);
//
//	/* Reinit SPI Structure */
//	SPI_InitStructure.SPI_Direction = (mode == SPI_Mode_Tx) ? SPI_Direction_1Line_Tx : SPI_Direction_1Line_Rx;
//	SPI_InitStructure.SPI_Mode = SPI_TRANS_MODE_1;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_MODE_1;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_MODE_1;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_InitStructure.SPI_CRCPolynomial = 7;
//	SPI_Init(SPI_MASTER, &SPI_InitStructure);
//	
//	/* Enable SPI */
//	SPI_Cmd(SPI_MASTER, ENABLE);
//}
//
//void SPI_CSDisable(void)
//{
//	/* Chip Select Pin High for Disable */
//	GPIO_SetBits(SPI_PORT_1, SPI_NSS_PIN_1);
//	/* Disable SPI */
//	SPI_Cmd(SPI_MASTER, DISABLE);
//}

#endif

#if DRIVER_SPI_EN > 0
int Driver_Spi_DMAInit(void)
{
	DMA_InitTypeDef  DMA_InitStructure;

	/* Enable peripheral clocks --------------------------------------------------*/
	/* Enable SPI_MASTER DMA clock */
	RCC_AHBPeriphClockCmd(SPI_MASTER_DMA_CLK, ENABLE);

	/* SPI_MASTER_Rx_DMA_Channel configuration ---------------------------------------------*/
	DMA_DeInit(SPI_MASTER_Rx_DMA_Channel);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_MASTER_DR_Base;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI_MASTER_Buffer_Rx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_MASTER_Rx_DMA_Channel, &DMA_InitStructure);

	/* SPI_MASTER_Tx_DMA_Channel configuration ---------------------------------------------*/
	DMA_DeInit(SPI_MASTER_Tx_DMA_Channel);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI_MASTER_Buffer_Tx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(SPI_MASTER_Tx_DMA_Channel, &DMA_InitStructure);

	/* Enable SPI_MASTER request */
	SPI_I2S_DMACmd(SPI_MASTER, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI_MASTER, SPI_I2S_DMAReq_Tx, ENABLE);

	/* Enable SPI_MASTER CRC calculation */
	SPI_CalculateCRC(SPI_MASTER, ENABLE);
	
	/* Enable DMA1 Channel4 and Channel5 */
	//DMA_Cmd(SPI_MASTER_Rx_DMA_Channel, ENABLE);
	//DMA_Cmd(SPI_MASTER_Tx_DMA_Channel, ENABLE);

	return 1;
}
#endif

#if (DRIVER_SPI_EN > 0)
int Driver_Spi_Read(Fd_t fd, unsigned char *pBuff, int len)
{
    uint32_t uiCnt;
    unsigned char *pDataIn;

	if(fd != 1)
		return -1;
    SPI_CSEnable();

    //
    // Initialize local variable.
    //
    pDataIn = pBuff;
    uiCnt = len;
	
    //
    // Reading loop
    //
	while(uiCnt--)
	{
		/* Wait for SPI_MASTER Tx buffer empty */
    	while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
		/* Send SPI_MASTER dummy write data */
		SPI_I2S_SendData(SPI_MASTER, SPI_MASTER_DUMMY_WRITE);

		/* Wait for SPI_MASTER data reception */
		while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
		/* Read SPI_MASTER received data */
		*pDataIn = (unsigned char)SPI_I2S_ReceiveData(SPI_MASTER);
		pDataIn++;
	}

	SPI_CSDisable();

    return len;
}
#endif

#if (DRIVER_SPI_EN > 0)
int Driver_Spi_Write(Fd_t fd, unsigned char *pBuff, int len)
{
	uint32_t uiCnt;
	unsigned char *pDataIn;

	if(fd != 1)
		return -1;
    SPI_CSEnable();

    //
    // Initialize local variable.
    //
    pDataIn = pBuff;
    uiCnt = len;

    //
    // Writing loop
    //
	while(uiCnt--)
	{
		/* Wait for SPI_MASTER Tx buffer empty */
    	while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
		/* Send SPI_MASTER data */
		SPI_I2S_SendData(SPI_MASTER, *pDataIn);

		/* Wait for SPI_MASTER data reception */
		while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
		/* Read SPI_MASTER dummy data */
		SPI_I2S_ReceiveData(SPI_MASTER);
		pDataIn++;
	}

	SPI_CSDisable();

    return len;
}
#endif

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
