#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

#define  TempVal   1
#define  HumVal    2

#define  WriterAddr	   0xB8			  //д������I2C��ַ0xB8+w(0)
#define  Funcode	   0x03			  //������
#define  StartAddr	   0x00			  //�Ĵ�����ʼ��ַ
#define  Regnum		   0x04			  //��ȡ�Ĵ�������
#define  ReadAddr	   0xB9			  //����������ַ0xB8+R(1)
#define  WokenTime	   2			  //����ʱ��    

//PB0
#define IIC_SCL_PORT              GPIOB
#define IIC_SCL_CLK               RCC_APB2Periph_GPIOB 
#define IIC_SCL_PIN               GPIO_Pin_0

//PB1
#define IIC_SDA_PORT              GPIOB
#define IIC_SDA_CLK               RCC_APB2Periph_GPIOB
#define IIC_SDA_PIN               GPIO_Pin_1

#define READ24C02				  (0xA1)
#define WRITE24C02				  (0xA0)

//IO��������	 
#define Set_IIC_SCL  {GPIO_SetBits(IIC_SCL_PORT,IIC_SCL_PIN);}
#define Clr_IIC_SCL  {GPIO_ResetBits(IIC_SCL_PORT,IIC_SCL_PIN);} 
#define Set_IIC_SDA  {GPIO_SetBits(IIC_SDA_PORT,IIC_SDA_PIN);}
#define Clr_IIC_SDA  {GPIO_ResetBits(IIC_SDA_PORT,IIC_SDA_PIN);} 
#define READ_SDA    (GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN))

/* private functions */
static void IIC_Start(void);
static u8 IIC_Wait_Ack(void);
static void IIC_Ack(void);
static void IIC_NAck(void);
static void IIC_Send_Byte(u8 txd);
static u8 IIC_Read_Byte(unsigned char ack);
static void delay_us(uint32_t i);

static void AT24C02_WriteByte(const uint8_t address, const uint8_t dat);
static uint8_t AT24C02_ReadByte(const uint8_t address);

//����IIC��ʼ�ź�
static void delay_us(uint32_t i)
{
	i = i * 3 * 7;	               //ʵ����ֵԼΪ7.1����	7*3
	while(i)
	{
		i--;
	}
}

//IO��������
//#define SDA_IN()  {GPIOB->MODER&=0XFFFFFF0F;GPIOB->MODER|=(uint32_t)8<<4;}
//#define SDA_OUT() {GPIOB->MODER&=0XFFFFFF0F;GPIOB->MODER|=3<<4;}
static void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}

static void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}
static void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	Set_IIC_SDA;	  	  
	Set_IIC_SCL;
	delay_us(15);  //��ʱ��� 10US	���10US����Դ󣬵��ǲ���С��5US 5US ��Ƶ�ʾ���100KHZ
 	Clr_IIC_SDA;    //START:when CLK is high,DATA change form high to low 
	delay_us(15);	//��ʱ��� 10US  �����delay_us(100) ������ҾͲ���˵���ˣ�����10US���ҵ�
	Clr_IIC_SCL;    //ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
static void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	Clr_IIC_SCL;
	Clr_IIC_SDA;  //STOP:when CLK is high DATA change form low to high
 	delay_us(15);
	Set_IIC_SCL;
	delay_us(15);
	Set_IIC_SDA;//����I2C���߽����ź�
	delay_us(15);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	Set_IIC_SDA;
	Clr_IIC_SCL
	SDA_IN();      //SDA����Ϊ����  
	delay_us(15);	   
	Set_IIC_SCL;
	delay_us(15);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			return 0;
		}
	}
	Clr_IIC_SCL;//ʱ�����0 	   
	return 1;  
} 
//����ACKӦ��
static void IIC_Ack(void)
{
	Clr_IIC_SCL;
	SDA_OUT();
	Clr_IIC_SDA;
	delay_us(15); // ����Ҳ��10US
	Set_IIC_SCL;
	delay_us(15);
	Clr_IIC_SCL;
}
//������ACKӦ��		    
static void IIC_NAck(void)
{
	Clr_IIC_SCL;
	SDA_OUT();
	Set_IIC_SDA;
	delay_us(15);
	Set_IIC_SCL;
	delay_us(15);
	Clr_IIC_SCL;
}					 				     		  
static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT();     
    for(t=0;t<8;t++)
    {              
        Clr_IIC_SCL;//����ʱ�ӿ�ʼ���ݴ���
		delay_us(15);
		if(txd&0x80)
		{
		  Set_IIC_SDA;
		}
		else
		{
		  Clr_IIC_SDA;
		}
		Set_IIC_SCL;
		delay_us(15);
		txd<<=1;
    }
	Clr_IIC_SCL;     //����ʱ�ӿ�ʼ���ݴ���	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        Clr_IIC_SCL; 
        delay_us(15);
		Set_IIC_SCL;
		delay_us(15);
        receive<<=1;
        if(READ_SDA)
		receive++;   
		delay_us(1);
    }
	Clr_IIC_SCL; 					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

void delay_ms(uint32_t i)
{

	  delay_us(i*1000);
	  //OSTimeDlyHMSM(0, 0, 0, i);

}

void woken(void)
{
       IIC_Start();	       
       IIC_Send_Byte(WriterAddr);
	   IIC_Wait_Ack();	  
       delay_ms(20);                 //����800US���Ͼ�OK 	                   
       IIC_Stop(); 
                
}

//��eeprom��ĵ�ַ�����Ҷ������ݵ�ַ��Ȼ�����Ƚ�
//������ĵ�ַ�����յأ����ݵ�ַĬ��������һ����ַ
//һ��������������ַ���ݣ�ǰ�����������ݣ�������ȡ���󱸷ݣ����������ҳ���ʼ��δд���������
E2PRETType E2P_ReadBKUP(const uint8_t address, uint8_t* dat)
{
   uint8_t c1,c2,c3;
   E2PRETType ret = E2P_OK;

   c1 = AT24C02_ReadByte(address);
   c2 = AT24C02_ReadByte(address+1);
   c3 = AT24C02_ReadByte(address+2);
   c3 = ~c3;

   if(c1 == c3)
   {//������ֵ���ϣ�pass
		*dat = c1;
		ret = E2P_OK; 
   }
   else
   {
	   if(c2 == c3)
	   {//������ֵ���ϣ�pass
	   		*dat = c2;
			ret = E2P_OK;
	   }
	   else if(c1 == c2)
	   {//c3���ܻ��������ǳ�ʼֵ����0xff
		   if((c3 == 0x00) && (c1 == 0xff))
		   {//˵���ǳ�ʼֵδ����ʼ��������Ĭ��ֵ
				ret = E2P_DEF;
	       }
	       else
		   {//����ֻ��c3����
				*dat = c1;
				ret = E2P_NG;
	       }
       }	
   }	

   return ret;

}

//���������ǵ�λ�ڵ͵�ַ����λ�ڸߵ�ַ���ҵ�ַ����
E2PRETType E2P_Read16BKUP(const uint8_t address, uint16_t *dat)
{
	uint8_t t = 0;
	E2PRETType ret = E2P_OK;

	E2P_ReadBKUP(address+3,&t);

	*dat = t;
	*dat = (*dat) << 8;

	E2P_ReadBKUP(address,&t);

	*dat += t;

	if(*dat == 0xffff)
		ret = E2P_DEF;
	else
		ret = E2P_OK;

	return ret;
}


E2PRETType E2P_WriteBKUP(const uint8_t address, const uint8_t dat)
{
	uint8_t c,ret;

	AT24C02_WriteByte(address,dat);
	AT24C02_WriteByte(address+1,dat);
	AT24C02_WriteByte(address+2,(~dat));

	ret = E2P_ReadBKUP(address,&c);

	if(ret == E2P_NG)
	{//��ֱ�Ӿ�ʧ����
	 	return E2P_NG;
	}

	if(c == dat)
	{//д�ɹ�
	    return E2P_OK;
	}
	else
	{//дʧ��
		return E2P_NG;
	}
}

E2PRETType E2P_Write16BKUP(const uint8_t address,const uint16_t dat)
{
	E2PRETType ret;
	uint8_t u8_low;
	uint8_t u8_high;

	u8_low = dat & 0xFF;
	u8_high = (dat >> 8) & 0xFF;

	ret = E2P_WriteBKUP(address,u8_low);
	ret = E2P_WriteBKUP(address+3,u8_high);

	return ret;
}

uint8_t AT24C02_ReadByte(const uint8_t address)
{
	uint8_t dat;
	IIC_Start();
	IIC_Send_Byte(WRITE24C02);
	IIC_Wait_Ack();
	//IIC_Ack();
	IIC_Send_Byte(address);
	IIC_Wait_Ack();
	//IIC_Ack();
	//IIC_Stop();
	IIC_Start();
	IIC_Send_Byte(READ24C02);
	IIC_Wait_Ack();
	//IIC_Ack();
	dat=IIC_Read_Byte(0);
	//IIC_NAck();
	IIC_Stop();

	return dat;
	
}

void AT24C02_WriteByte(const uint8_t address, const uint8_t dat)
{
 //uint8_t temp;
 IIC_Start();
 IIC_Send_Byte(WRITE24C02);
 IIC_Wait_Ack();     
 //IIC_Ack();
 
 IIC_Send_Byte(address); 
 IIC_Wait_Ack();  
 //IIC_Ack();
 
 IIC_Send_Byte(dat);    
 IIC_Wait_Ack();
 //IIC_NAck();

 IIC_Stop();
 delay_ms(10);

}

void E2P_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;			//�������
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
	
	Set_IIC_SCL;
	Set_IIC_SDA;
}


