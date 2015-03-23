#include "stm32f10x.h"
#include "stm32f10x_wtd_drivers.h"

#define  TempVal   1
#define  HumVal    2

#define  WriterAddr	   0xB8			  //写传感器I2C地址0xB8+w(0)
#define  Funcode	   0x03			  //功能码
#define  StartAddr	   0x00			  //寄存器起始地址
#define  Regnum		   0x04			  //读取寄存器个数
#define  ReadAddr	   0xB9			  //读传感器地址0xB8+R(1)
#define  WokenTime	   2			  //唤醒时间    

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

//IO操作函数	 
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

//产生IIC起始信号
static void delay_us(uint32_t i)
{
	i = i * 3 * 7;	               //实际数值约为7.1左右	7*3
	while(i)
	{
		i--;
	}
}

//IO方向设置
//#define SDA_IN()  {GPIOB->MODER&=0XFFFFFF0F;GPIOB->MODER|=(uint32_t)8<<4;}
//#define SDA_OUT() {GPIOB->MODER&=0XFFFFFF0F;GPIOB->MODER|=3<<4;}
static void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}

static void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
}
static void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	Set_IIC_SDA;	  	  
	Set_IIC_SCL;
	delay_us(15);  //延时大概 10US	这个10US你可以大，但是不能小于5US 5US 其频率就是100KHZ
 	Clr_IIC_SDA;    //START:when CLK is high,DATA change form high to low 
	delay_us(15);	//延时大概 10US  你这个delay_us(100) 下面的我就不作说明了，都是10US左右的
	Clr_IIC_SCL;    //钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
static void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	Clr_IIC_SCL;
	Clr_IIC_SDA;  //STOP:when CLK is high DATA change form low to high
 	delay_us(15);
	Set_IIC_SCL;
	delay_us(15);
	Set_IIC_SDA;//发送I2C总线结束信号
	delay_us(15);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	Set_IIC_SDA;
	Clr_IIC_SCL
	SDA_IN();      //SDA设置为输入  
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
	Clr_IIC_SCL;//时钟输出0 	   
	return 1;  
} 
//产生ACK应答
static void IIC_Ack(void)
{
	Clr_IIC_SCL;
	SDA_OUT();
	Clr_IIC_SDA;
	delay_us(15); // 这里也是10US
	Set_IIC_SCL;
	delay_us(15);
	Clr_IIC_SCL;
}
//不产生ACK应答		    
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
        Clr_IIC_SCL;//拉低时钟开始数据传输
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
	Clr_IIC_SCL;     //拉低时钟开始数据传输	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
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
       delay_ms(20);                 //大于800US以上就OK 	                   
       IIC_Stop(); 
                
}

//读eeprom里的地址，并且读出备份地址，然后做比较
//参数里的地址是首收地，备份地址默认是其下一个地址
//一个数据用三个地址备份，前两个正常备份，第三个取反后备份，这样可以找出初始化未写入这种情况
E2PRETType E2P_ReadBKUP(const uint8_t address, uint8_t* dat)
{
   uint8_t c1,c2,c3;
   E2PRETType ret = E2P_OK;

   c1 = AT24C02_ReadByte(address);
   c2 = AT24C02_ReadByte(address+1);
   c3 = AT24C02_ReadByte(address+2);
   c3 = ~c3;

   if(c1 == c3)
   {//两个数值符合，pass
		*dat = c1;
		ret = E2P_OK; 
   }
   else
   {
	   if(c2 == c3)
	   {//两个数值符合，pass
	   		*dat = c2;
			ret = E2P_OK;
	   }
	   else if(c1 == c2)
	   {//c3可能坏，或者是初始值都是0xff
		   if((c3 == 0x00) && (c1 == 0xff))
		   {//说明是初始值未被初始化，返回默认值
				ret = E2P_DEF;
	       }
	       else
		   {//可能只是c3损坏了
				*dat = c1;
				ret = E2P_NG;
	       }
       }	
   }	

   return ret;

}

//读出必须是低位在低地址，高位在高地址，且地址连续
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
	{//读直接就失败了
	 	return E2P_NG;
	}

	if(c == dat)
	{//写成功
	    return E2P_OK;
	}
	else
	{//写失败
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;			//推挽输出
	GPIO_Init(IIC_SDA_PORT, &GPIO_InitStructure);
	
	Set_IIC_SCL;
	Set_IIC_SDA;
}


