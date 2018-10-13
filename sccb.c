#include "sccb.h"

#define SDA_H  gpio_bit_set(GPIOB, GPIO_PIN_13)  
#define SDA_L  gpio_bit_reset(GPIOB, GPIO_PIN_13)   

#define SCL_H  gpio_bit_set(GPIOB, GPIO_PIN_14)  
#define SCL_L  gpio_bit_reset(GPIOB, GPIO_PIN_14)  

#define RST_H  gpio_bit_set(GPIOB, GPIO_PIN_15)  
#define RST_L  gpio_bit_reset(GPIOB, GPIO_PIN_15)  

#define PWDN_H  gpio_bit_set(GPIOB, GPIO_PIN_12)  
#define PWDN_L  gpio_bit_reset(GPIOB, GPIO_PIN_12) 

//�ڿ�©���ģʽ�£��Զ˿�����״̬�Ĵ����Ķ����ʽ�����I/O��״̬����� ����ҪΪ�˶�ȡ���ݣ�ר�Ž�I/0�������Ϊ����
#define  SDA_read           gpio_input_bit_get(GPIOB, GPIO_PIN_13) 


void I2C_GPIO_Configuration(void)
{
     gpio_deinit(GPIOB);
  /* ģ��I2C PB13--SDA  PB14-SCL */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_13); //�趨GPI0B�������,�����ȶ�
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_13);//GPIOBP OD,�ٶ����50MHZ
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_14);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_14);
	//��ʼ������ͷRST PB4    PWDN PB5
	  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_15);
	  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_15);
	  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_12);
	  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
	 /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
}
void I2C_delay(void)                         //Delay 
{
    uint32_t t=300*13.5;                        //300/8000=37.5us�� �����108MHz,�����13.5
    
	 while(t--);
	
}

void nops1ms(void) //1ms
{
    uint32_t t=8000*13.5;     //8000/8000=1ms�� �����108MHz,�����13.5 
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
  SCL_H;                                  //��λ״̬
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
	SCL_H;//�ߵ�ƽ��Ч
	I2C_delay();//��ʱ
	//�鿴��ʱSDA�Ƿ�������ߵ�ƽ��
	if(!SDA_read)
	{
		printf("\r\nSDA��Ϊ�͵�ƽ������æ���˳�\r\n");
		return DISABLE;//SDA����æ���˳�
	}
	//����һ���½��أ��½����ǿ�ʼ�ı�־
	SDA_L;
	I2C_delay();
	//�鿴��ʱSDA�Ѿ���Ϊ�͵�ƽ
	if(SDA_read)
	{
		printf("\r\nSDA��Ϊ�ߵ�ƽ�����߳����˳�\r\n");
		return DISABLE;//SDA����æ���˳�
	}
	SCL_L;
	return ENABLE;
}

void I2C_Stop(void)
{
	
	SCL_L;
	//����һ�������أ��������ǽ����ı�־
	SDA_L;	
	SCL_H;//�ߵ�ƽ��Ч
	I2C_delay();//��ʱ
	SDA_H;
	I2C_delay();
}

//������Ӧ���ź�,�����ѵھ�λ�øߣ��ӻ��������ͱ�ʾӦ��
void I2C_Ack()
{
	SCL_L;
	SDA_L;//�õ�
	I2C_delay();   //ע����ʱʱ��Ӧ�ô���4΢�룬����λ��Ҳ�����
	SCL_H;
	I2C_delay();
	SCL_L;
}
 
//�����ķ�Ӧ���ź�,�ӻ��ѵھ�λ�øߣ������������ͱ�ʾ��Ӧ��
void I2C_NoAck()
{
	SCL_L;
	I2C_delay();
	SDA_H;//�ø�
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
	while(SDA_read)//�ӻ�δӦ����Ӧ�𣬻����͵ھ�λ
	{
		time++;
		if(time > 250)
		{
			//��Ӧ��ʱ�����Է�����ֹ�źţ����򣬸��϶�дģʽ�²����Խ��еڶ��׶�
			//SCCB_Stop();
			
			SCL_L;
			return DISABLE;
		}
	}
	SCL_L;
	return ENABLE;
}

//I2Cдһ���ֽ�
void I2C_SendByte(uint8_t Data)
{
  uint8_t cnt;

  for(cnt=0; cnt<8; cnt++)
  {
    SCL_L;                                 //SCL��(SCL��ʱ���仯SDA)
    I2C_delay();
 
    if(Data & 0x80)
    {
      SDA_H;                              //SDA�ߣ������λ��ʼд��
    }
    else
    {
      SDA_L;                               //SDA��
    }
    Data <<= 1;
    SCL_H;                                //SCL��(��������)
    I2C_delay();
  }
  SCL_L;                                   //SCL��(�ȴ�Ӧ���ź�)
  I2C_delay();
}
 
//I2C��ȡһ���ֽ�
uint8_t I2C_ReadByte(uint8_t ack)
{
  uint8_t cnt;
  uint8_t data;
	
  for(cnt=0; cnt<8; cnt++)
  {
    SCL_L;                                 //SCL��
    I2C_delay();
		
    SCL_H;                                //SCL��(��ȡ����)
    data <<= 1;
    if(SDA_read)
    {
      data |= 0x01;                              //SDA��(������Ч)
    }
    I2C_delay();
  }
  //����Ӧ���źţ�Ϊ�ʹ���Ӧ�𣬸ߴ����Ӧ��
  if(ack == 1)
  {
     I2C_NoAck();
  }
  else
  {
     I2C_Ack();
  }
  return data;                                   //��������
}

uchar SCCBReadByte(uint Addr)
{
	uchar data;
	//I2C_Initializes();
	I2C_Start();
	I2C_SendByte(0x78);
	
	I2C_GetAck();//�����ں��Ƿ�Ӧ��
	I2C_SendByte((uint8_t)((Addr>>8) & 0xFF) );
	I2C_GetAck();//�����ں��Ƿ�Ӧ��
  I2C_SendByte( (uint8_t)(Addr & 0xFF) );
	I2C_GetAck();//�����ں��Ƿ�Ӧ��
	
	I2C_Start();//�����ں��Ƿ�Ӧ��
	I2C_SendByte(0x79);
	
	I2C_GetAck();//�����ں��Ƿ�Ӧ��
	data = I2C_ReadByte(1); 
  I2C_Stop();
	
	return data;	
}
