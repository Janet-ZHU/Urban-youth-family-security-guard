#include "sccb.h"

#define SDA_H  gpio_bit_set(GPIOB, GPIO_PIN_13)  
#define SDA_L  gpio_bit_reset(GPIOB, GPIO_PIN_13)   

#define SCL_H  gpio_bit_set(GPIOB, GPIO_PIN_14)  
#define SCL_L  gpio_bit_reset(GPIOB, GPIO_PIN_14)  

#define RST_H  gpio_bit_set(GPIOB, GPIO_PIN_15)  
#define RST_L  gpio_bit_reset(GPIOB, GPIO_PIN_15)  

#define PWDN_H  gpio_bit_set(GPIOB, GPIO_PIN_12)  
#define PWDN_L  gpio_bit_reset(GPIOB, GPIO_PIN_12) 

//在开漏输出模式下，对端口输入状态寄存器的读访问将返回I/O的状态，因此 不需要为了读取数据，专门将I/0由输出设为输入
#define  SDA_read           gpio_input_bit_get(GPIOB, GPIO_PIN_13) 


void I2C_GPIO_Configuration(void)
{
     gpio_deinit(GPIOB);
  /* 模拟I2C PB13--SDA  PB14-SCL */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_13); //设定GPI0B上拉输出,上拉稳定
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_13);//GPIOBP OD,速度最大50MHZ
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_14);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_14);
	//初始化摄像头RST PB4    PWDN PB5
	  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_15);
	  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_15);
	  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_12);
	  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
	 /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
}
void I2C_delay(void)                         //Delay 
{
    uint32_t t=300*13.5;                        //300/8000=37.5us； 如果是108MHz,则乘以13.5
    
	 while(t--);
	
}

void nops1ms(void) //1ms
{
    uint32_t t=8000*13.5;     //8000/8000=1ms； 如果是108MHz,则乘以13.5 
	  while(t--);
}

void delay_50ms(void) //50ms
{
  uint t=50;   
	for(t=0;t<50;t++)
	nops1ms();
}


void init_camera(void)
{  
	 uint i;
	 RST_L;
	 PWDN_H;
	 for(i=0;i<10;i++)
	 nops1ms();
	 PWDN_L;	    
	 for(i=0;i<10;i++)
	 nops1ms();
	 RST_H;
}

void I2C_Initializes(void)
{
  I2C_GPIO_Configuration();
	init_camera();
	delay_50ms();
  SCL_H;                                  //置位状态
  SDA_H;
	nops1ms();
}

void Delay(void)                        //Delay for 2 milliseconds.(2000us)
{                                       //2000/1/24.5=50000
   nops1ms();
   nops1ms();
}

int I2C_Start(void)
{
	
	SDA_H;
	SCL_H;//高电平有效
	I2C_delay();//延时
	//查看此时SDA是否就绪（高电平）
	if(!SDA_read)
	{
		printf("\r\nSDA线为低电平，总线忙，退出\r\n");
		return DISABLE;//SDA总线忙，退出
	}
	//制造一个下降沿，下降沿是开始的标志
	SDA_L;
	I2C_delay();
	//查看此时SDA已经变为低电平
	if(SDA_read)
	{
		printf("\r\nSDA线为高电平，总线出错，退出\r\n");
		return DISABLE;//SDA总线忙，退出
	}
	SCL_L;
	return ENABLE;
}

void I2C_Stop(void)
{
	
	SCL_L;
	//制造一个上升沿，上升沿是结束的标志
	SDA_L;	
	SCL_H;//高电平有效
	I2C_delay();//延时
	SDA_H;
	I2C_delay();
}

//主机的应答信号,主机把第九位置高，从机将其拉低表示应答
void I2C_Ack()
{
	SCL_L;
	SDA_L;//置低
	I2C_delay();   //注意延时时间应该大于4微秒，其他位置也是如此
	SCL_H;
	I2C_delay();
	SCL_L;
}
 
//主机的非应答信号,从机把第九位置高，主机将其拉低表示非应答
void I2C_NoAck()
{
	SCL_L;
	I2C_delay();
	SDA_H;//置高
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
}

uint8_t I2C_GetAck(void)
{
  uint8_t time = 0;
	
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	while(SDA_read)//从机未应答，若应答，会拉低第九位
	{
		time++;
		if(time > 250)
		{
			//不应答时不可以发出终止信号，否则，复合读写模式下不可以进行第二阶段
			//SCCB_Stop();
			
			SCL_L;
			return DISABLE;
		}
	}
	SCL_L;
	return ENABLE;
}

//I2C写一个字节
void I2C_SendByte(uint8_t Data)
{
  uint8_t cnt;

  for(cnt=0; cnt<8; cnt++)
  {
    SCL_L;                                 //SCL低(SCL低时，变化SDA)
    I2C_delay();
 
    if(Data & 0x80)
    {
      SDA_H;                              //SDA高，从最低位开始写起
    }
    else
    {
      SDA_L;                               //SDA低
    }
    Data <<= 1;
    SCL_H;                                //SCL高(发送数据)
    I2C_delay();
  }
  SCL_L;                                   //SCL低(等待应答信号)
  I2C_delay();
}
 
//I2C读取一个字节
uint8_t I2C_ReadByte(uint8_t ack)
{
  uint8_t cnt;
  uint8_t data;
	
  for(cnt=0; cnt<8; cnt++)
  {
    SCL_L;                                 //SCL低
    I2C_delay();
		
    SCL_H;                                //SCL高(读取数据)
    data <<= 1;
    if(SDA_read)
    {
      data |= 0x01;                              //SDA高(数据有效)
    }
    I2C_delay();
  }
  //发送应答信号，为低代表应答，高代表非应答
  if(ack == 1)
  {
     I2C_NoAck();
  }
  else
  {
     I2C_Ack();
  }
  return data;                                   //返回数据
}

uchar SCCBReadByte(uint Addr)
{
	uchar data;
	//I2C_Initializes();
	I2C_Start();
	I2C_SendByte(0x78);
	
	I2C_GetAck();//不必在乎是否应答
	I2C_SendByte((uint8_t)((Addr>>8) & 0xFF) );
	I2C_GetAck();//不必在乎是否应答
  I2C_SendByte( (uint8_t)(Addr & 0xFF) );
	I2C_GetAck();//不必在乎是否应答
	
	I2C_Start();//不必在乎是否应答
	I2C_SendByte(0x79);
	
	I2C_GetAck();//不必在乎是否应答
	data = I2C_ReadByte(1); 
  I2C_Stop();
	
	return data;	
}
