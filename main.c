#include "main.h"

//����Ӧ�ó�����Ҫ��main.c �� gd32f3x0_it.c�ĵ�ĩβ��ETI�жϴ������ж��壬������GD32F350�ٷ��̼��⣬
//������ʹ����GPIO��TIMER����RTC��DMA���ⲿ�жϣ�I2C��FLASH, UART��PLL��ģ�顣
//����ͷ��������SCCB/ov5640��
//���������ѧϰʹ�ã�EEworld ID:��ýѧ��, ��ͨ��EEworld��̳�������ԣ����ۡ�
//���������������״��¹�˾������GD32F350 arm��������
//��󣬸�л���״��¹�˾��EEworld��̳�ܹ��ٰ���δ��´�����


//���º���ֻ��mian.c���õ�������������main.h�ж��壬��������.c�ļ�����
void rtc_setup(void);
void rtc_show_time(void);
void rtc_pre_config(void);
void image_process(void);//ͼ����
void serial_LCD(uint8_t x);
void FLASH_ProgramBytes(uint32_t Address, uint8_t *Buffer, uint16_t ByteCount);
void rain_and_rsd_senser(void);

// ������LED1 ��Ӧ  PB10 ��Ӧ������LED4���ϱߣ� LED2  PB8  LED5�м䣬 LED3 ���±ߵ���һ��PB9

//ʵʱʱ�Ӷ���
#define RTC_CLOCK_SOURCE_IRC40K 
#define BKP_VALUE    0x32F0
rtc_parameter_struct   rtc_initpara;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;


uint8_t flag=0;//flag =1,����ͼ����flag = 0,�����д���	
uint8_t flag1=0;//��һ�����ݣ�data1��data2ͬʱ��ȡ��֮��data1��ȡ�µģ�data2��ȡdata1��	
uint8_t flag2=0;//���ο����׶εļ�����ͼ���жϣ���ֹ��������ʾ

#define OV5640_DEVICE_ADDRESS    0x78  // 0111 1000 ov5640 I2C�豸��ַ������ʱ��0x78��д��ʱ��0x79
//uint8_t v_switch = 1,h_switch = 0;
uint16_t vs_i=0,hs_i=0; //begin ��ʼ�ɼ�ͼ��
uint32_t p_i = 0, p_j = 0;
__IO uint16_t data_buf[4050]={0};	//YUV 4;2;2  YUYV  ÿ��Y��һ��ɫ���ź�Ϊһ�����أ����160*120Ӧ����� 38400��byte;
uint8_t image_buf[4050]={0};	//���81*50��8-bit�Ҷ�ͼ��;

uint8_t data0[450] ={0};
uint8_t data1[450] ={0};//�洢��ǰ֡ͼ��
uint8_t data2[450] ={0};//�洢��֡ͼ��
const uint32_t data3[1013] ={0};//���ָ澯ʱ���洢�澯ͼ����flash��data3�� //4050/4 = 1013


//����DMA��Timer��ʼ���ṹ��
dma_parameter_struct DMA_InitStructure;
timer_parameter_struct TIM_TimeBaseStructure;
timer_ic_parameter_struct	TIM_ICInitStructure;


#define B2  gpio_input_bit_get(GPIOA, GPIO_PIN_0)//����
#define B3  gpio_input_bit_get(GPIOB, GPIO_PIN_7) //����
uint8_t B2_i = 0,B3_i =0; //������ʾ����

#define Sensor1  gpio_input_bit_get(GPIOC, GPIO_PIN_0)//���͵�
#define Sensor2  gpio_input_bit_get(GPIOC, GPIO_PIN_1) //���


//ͼ�����㷨���̶�����ͷ�󣬵����˳���ʱ����澯
void image_process(void)
{
	 uint16_t  i, j = 0, k = 0, dot_diff=0;
	 //exti_interrupt_disable(EXTI_5);	 flag == 1 ʱ�Ѿ��رճ��ж�
   uint32_t image_diff = 0;
	 for( i=0;i<4050;i++){	 //��ȡ����ͼ���������
			 image_buf[i] = (uint8_t)(data_buf[i]>>6);
			 j++;
			 if(j == 9) //ÿ��ȡ9�����ݵ�
			 {
				j=0;
				data0[k++] = image_buf[i];
			 }	
		 }				 
	
	 if(!flag1){ //flag1 == 0, ͬʱ��ȡdata0����
		 for( i=0;i<450;i++){	 
				data1[i] = data0[i];  
			  data2[i] = data0[i]; 
		 }
		 //flag1 = 1;
    }
	 else if(flag1 == 1)//flag1 == 1, ֻ��data1��ȡdata0����
	 {
			 for( i=0;i<450;i++){	 
			   data1[i] = data0[i];  
			 }
		   //flag1 = 2; 
	 }
	 else{ //flag1 == 2, data2�ȴ�ȡdata1���ݣ�ͬʱdata1��ȡdata0����,
			 for( i=0;i<450;i++){	
         data2[i] = data1[i]; 				 
			   data1[i] = data0[i];  
			 } 
	 }
	 
	 for(i=0; i<450;i++)
	{
		if(data1[i] >= data2[i])
	  dot_diff = data1[i] - data2[i];
		else
		dot_diff = data2[i] - data1[i];	
		if(dot_diff > 20) //�ж�����ͼ���У���ͬ�ĵ��ж��ٸ�������ֵӰ��ͼ��ʶ��׼ȷ�ȣ������ʵ������Ż�
		image_diff += 1;
	}
	if(image_diff > 40) //����ͼ���в�ͬ��������������ֵ����Ϊ���˴��룬�����ʵ������Ż�
	{
		if(flag2<=2) //����ǰ�����ж�
		{
			flag2++;
			serial_LCD(0);//��ʾ����
			serial_LCD(2);
			serial_LCD(4);
		}
		if(flag2 == 3)
		{
		printf("\r\n �澯�����˴���!  "); 
		rtc_show_time();
		serial_LCD(5);//��ʾ�澯
//		
//		for(i=0; i<4050;i++)
//		{
//			FLASH_ProgramBytes(*data3, image_buf, 4050);//�洢�澯ͼ��
//			//�����������
//		}
		}
	}
	else 
	{
		//printf("\r\n ���� \r\n");
		//serial_LCD(4);
	}
   exti_interrupt_enable(EXTI_5);	//������ϣ�ʹ�ܳ��ж�
}



void nops500ms_108M(void) //1ms
{
    uint32_t t=54000000;   
	  while(t--);
}
//USART0��ӦPA2,PA3

int main(void)
{
	  rcu_periph_clock_enable(RCU_GPIOA);//��Ҫ���������ڲ���ʹ��
		gd_eval_com_init(EVAL_COM1,115200);
	  //�������������ʱֻ����ʾӢ�ģ�������������ǲ���Ӣ��ʹ��
	  printf(" Welcome you ! Now this is buletooth connection.\n");
    printf(" Start the moniter mode... \n");
    printf(" Next, transfer to PC using 115200, buletooth will waiting... \n");
	
	 //������Щ�ǵ���ʱʹ��
	
	  printf("\r\n***���������ͥ������ʿ***");
		printf("\r\n");
	  printf("\r\n       Welcome you!   \r\n");
		
		rtc_setup();//����RTCʵʱʱ��
	  show_system_clk();//��ʾ��ǰʱ��
    rcu_periph_clock_enable(RCU_CFGCMP);//need to be started
				
		OV5640_HW_Init();//��ʼ������ͷ��YUV420�����YYYY/YUYV ��һ�����YYYYYY �ڶ������YUYV ���к�����4��2��0
    OV5640_OutSize_Set(0,0,160,120);//��������ߴ� 
		
		gd_eval_led_init(LED1);
		gd_eval_led_init(LED2);
		gd_eval_led_init(LED3);

		gd_eval_led_off(LED1);
		gd_eval_led_off(LED2);
		gd_eval_led_off(LED3);
		
		gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);//��ʼ������B2����Ϊ�ж��Ƿ�����������δ������ı�־��ӦLED2
		gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);//��ʼ������B3����Ϊ�ж��Ƿ������������͵紫�����ı�־��ӦLED3
		
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_0);//���͵� ���1�����Ӧ�����壬��ģ����8s���ҵ��ӳ�
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_1);//��δ����������0 ���������


		printf("\r\n�������״̬��");
		printf("\r\n***���͵紫�����������***");
		printf("\r\n***����ͨ��ģ���������***");
	  printf("\r\n***��δ�����ģ���������***");
		
		if((0x56 == SCCBReadByte(0x300A)) && (0x40 == SCCBReadByte(0x300B)))
    {
			printf("\r\n***����ͷ������***");
		  gd_eval_led_on(LED1);//LED1����������ͷ��ʼ�����
		}
		else
	  printf("\r\n δ��⵽����ͷ����������ͷ�쳣��");
		
		printf("\r\n");
    printf("\r\n***ϵͳ�Լ���ɣ�������ģʽ***");
    printf("\r\n");
		rtc_show_time();
		
		//ͼ���жϡ����жϷֱ���PA5 PA6 ��Ϊ�ⲿ�ж����룬�ɶ�ʱ����׽PCLK(Timer1_CH1),����DMA������GPIOC�������ݵ��ڴ���
		// ��ʱ�������DMA ���������
    config_dma();//��ʼ��DMA
	  config_timer();//��ʼ��timer
	  config_interrupt();//��ʼ���ж�
		
	 
		
    while (1){
			   if(flag)//ͼ����
				 {
           image_process();
					 flag = 0;
				 }
				 rain_and_rsd_senser();//��δ���������
  }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));

    return ch;
}

void config_interrupt()
{
    /* enable the key clock */
    rcu_periph_clock_enable(RCU_GPIOA); //��ov5640��ʼ�����Ѿ�������
	  rcu_periph_clock_enable(RCU_CFGCMP);//����������ͷ����
    /* configure button pin as input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_5); //vs
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6); //HREF
 
	
		/* enable and set EXTI interrupt to the lowest priority */
    nvic_irq_enable(EXTI4_15_IRQn, 2U, 0U);
		/* connect key EXTI line to key GPIO pin */
		syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN5);
	  syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN6);



		/* configure EXTI line */
	  exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_RISING);//������
		exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_RISING);//������
  // exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_RISING);//������
	
		exti_interrupt_flag_clear(EXTI_5);
	  exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_disable(EXTI_6);//�ȹر����ж� ���ж�Ϊ12
	
}

void config_dma(void)
{
	 rcu_periph_clock_enable(RCU_DMA);
	 DMA_InitStructure.periph_addr  = (uint32_t) &GPIO_ISTAT(GPIOC);//��ȷ���費��Ҫ��&
	 DMA_InitStructure.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	 DMA_InitStructure.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	 DMA_InitStructure.memory_addr = (uint32_t) data_buf;//��ȷ���費��Ҫ��&
	 DMA_InitStructure.memory_width = DMA_MEMORY_WIDTH_16BIT;
   DMA_InitStructure.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	 DMA_InitStructure.direction  = DMA_PERIPHERAL_TO_MEMORY;
   DMA_InitStructure.number = 4050;
	 DMA_InitStructure.priority = DMA_PRIORITY_ULTRA_HIGH;
	
	 dma_init(DMA_CH2, DMA_InitStructure);
	 //dma_circulation_disable(DMA_CH2);
	 dma_circulation_enable(DMA_CH2);
	// dma_channel_enable(DMA_CH2);
}
void config_timer(void)
{
	 //PA1����
	 gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_1);//PCLK
	 gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_1);
	//TIM1
	rcu_periph_clock_enable(RCU_TIMER1);
	TIM_TimeBaseStructure.prescaler = 0;
	TIM_TimeBaseStructure.alignedmode = TIMER_COUNTER_EDGE;
	TIM_TimeBaseStructure.counterdirection = TIMER_COUNTER_UP;
	TIM_TimeBaseStructure.period = 2;// ��0~200
	TIM_TimeBaseStructure.clockdivision = TIMER_CKDIV_DIV1;
	TIM_TimeBaseStructure.repetitioncounter = 0;
	timer_init(TIMER1, &TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.icpolarity = TIMER_IC_POLARITY_RISING;
	TIM_ICInitStructure.icselection = TIMER_IC_SELECTION_DIRECTTI;
	TIM_ICInitStructure.icprescaler = TIMER_IC_PSC_DIV2;//����ͷPCLK2��Ƶ��һ��ȡ80����
	TIM_ICInitStructure.icfilter = 0;
	
	timer_input_capture_config(TIMER1,TIMER_CH_1, &TIM_ICInitStructure);
	timer_dma_enable(TIMER1, TIMER_DMA_CH1D);
	timer_enable(TIMER1);
}

void rtc_pre_config(void)
{
    #if defined (RTC_CLOCK_SOURCE_IRC40K) 
          rcu_osci_on(RCU_IRC40K);
          rcu_osci_stab_wait(RCU_IRC40K);
          rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);
  
          prescaler_s = 0x18F;
          prescaler_a = 0x63;
    #elif defined (RTC_CLOCK_SOURCE_LXTAL)
          rcu_osci_on(RCU_LXTAL);
          rcu_osci_stab_wait(RCU_LXTAL);
          rcu_rtc_clock_config(RCU_RTC_LXTAL);
          prescaler_s = 0xFF;
          prescaler_a = 0x7F;
    #else
    #error RTC clock source should be defined.
    #endif /* RTC_CLOCK_SOURCE_IRC40K */

    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}
void rtc_setup(void)
{
    /* setup RTC time value */
    	/* enable PMU clock */
    rcu_periph_clock_enable(RCU_PMU);
    /* enable the access of the RTC registers */
    pmu_backup_write_enable();
		rtc_pre_config();
	
    rtc_initpara.rtc_factor_asyn = prescaler_a;
    rtc_initpara.rtc_factor_syn = prescaler_s;
    rtc_initpara.rtc_year = 0x18;
    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
    rtc_initpara.rtc_month = RTC_OCT;
    rtc_initpara.rtc_date = 0x30;
    rtc_initpara.rtc_display_format = RTC_24HOUR;
    rtc_initpara.rtc_am_pm = RTC_AM;

    rtc_initpara.rtc_hour = 0x15;
    rtc_initpara.rtc_minute = 0x50;
    rtc_initpara.rtc_second = 0x00;
    
    /* RTC current time configuration */
    if(ERROR == rtc_init(&rtc_initpara)){    
        printf("\n\r** RTC time configuration failed! **");
    }else{
        //printf("\n\r** RTC time configuration success! **");
			  printf("  Today: 20%0.2x��%0.2x��%0.2x�� \n\r", \
          rtc_initpara.rtc_year, rtc_initpara.rtc_month, rtc_initpara.rtc_date);
        rtc_show_time();
			
        RTC_BKP0 = BKP_VALUE;
    }
		  
}
/*!
    \brief      display the current time
*/
void rtc_show_time(void)
{
    uint32_t time_subsecond = 0;
    uint8_t subsecond_ss = 0,subsecond_ts = 0,subsecond_hs = 0;

    rtc_current_time_get(&rtc_initpara);  

    /* get the subsecond value of current time, and convert it into fractional format */
    time_subsecond = rtc_subsecond_get();
    subsecond_ss=(1000-(time_subsecond*1000+1000)/400)/100;
    subsecond_ts=(1000-(time_subsecond*1000+1000)/400)%100/10;
    subsecond_hs=(1000-(time_subsecond*1000+1000)/400)%10;
    
    printf("- Current time: %0.2x:%0.2x:%0.2x .%d%d%d \n\r", \
          rtc_initpara.rtc_hour, rtc_initpara.rtc_minute, rtc_initpara.rtc_second,\
          subsecond_ss, subsecond_ts, subsecond_hs);
}




//��ϵͳʱ��Դ�л���8MHz
void config_system_clk_8M(void)//���к���
{
		//rcu_deinit();//��Ҫ��ʼ��RCU�����������
		RCU_CFG0 &= 0xFFFFFFFC;//��SCS��λΪ00
		while(RCU_CFG0&0x0000000C);//�ж�SCSSΪ00��ת�����
		//gd_eval_com_init(EVAL_COM1);
		printf("\r\nRCU_CFG0��%x",RCU_CFG0);
		show_system_clk();
}

void config_system_clk_108M(void)//���к���
{
	  printf("\r\n�ȴ�pll����..");
	  RCU_CTL0 &= ~BIT(24); //�ر�PLLEN
		RCU_CTL0 &= ~BIT(25); //��PLL�ȶ���־����
		RCU_CFG0 &= ~BIT(19);//BIT19 �任��ƵΪ20
		RCU_CTL0 |= BIT(24); //ʹ��PLLEN
		while(!(RCU_CTL0&(BIT(25))));
		RCU_CFG0 |= BIT(1);//��SCS��λΪ10
		while(!(RCU_CFG0&0x0000000C));//�ж�SCSSΪ10��ת�����
		//gd_eval_com_init(EVAL_COM1);
		show_system_clk();
}

void show_system_clk(void)//�Խ�����
{
	 if(8 == rcu_system_clock_source_get())
	 printf("\r\nĿǰ��ϵͳʱ��ԴΪ��CK_PLL");//10 00 - CK_PLL;0000 CK_IRC8M
   printf("\r\nĿǰ��ϵͳʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_SYS));
   printf("\r\nĿǰ��AHBʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_AHB));
	 printf("\r\nĿǰ��CK_APB1ʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_APB1));
	 printf("\r\nĿǰ��CK_APB2ʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_APB2));
	 printf("\r\nĿǰ��CK_ADCʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_ADC)); //28M/2
	 printf("\r\nĿǰ��CK_CECʱ��Ƶ��Ϊ��%dHz",rcu_clock_freq_get(CK_CEC));
	 printf("\r\nĿǰ��CK_USARTʱ��Ƶ��Ϊ��%dHz  ",rcu_clock_freq_get(CK_USART));
}


void FLASH_ProgramBytes(uint32_t Address, uint8_t *Buffer, uint16_t ByteCount)		//Flashд����8λ�ֽ�
{
    uint16_t i = 0;														//ѭ������
    fmc_unlock();;
    while(i<ByteCount)			//ѭ���ֽڴ���
    {
	   fmc_word_program(Address, *(u32*)Buffer);
        i = i+4;													//�ֽ���+4
        Address = Address + 4;										//��ַ+4
        Buffer = Buffer + 4;										//Ҫд��������ֽ�+4
			  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    }
    fmc_lock();
}


//************************************************************************************
//***����ֻ��demo,��ʵ��Ӧ�õ��У����Խ��������������������ж��д�����������
//************************************************************************************
void rain_and_rsd_senser(void)//��δ����������͵紫����
{
	
	//�����������
	if(!B2){ //Ϊ���Է��㰴��B2�ż�����͵�
	 gd_eval_led_on(LED2);
   if(Sensor1&&!B2_i)//��Ϊ�����Ը������ˣ��������������һֱΪ��Ч״̬
	 {	 
		 printf("\r\n ���͵紫����̽�⵽��\r\n");
		 rtc_show_time();
		 serial_LCD(1);
	 }
	 if(!Sensor1&&!B2_i)
	 {
		 serial_LCD(0);//��ӡ������LCD
	 }
	 B2_i = 1;
	 B3_i = 0;
 }
	else
	{
		gd_eval_led_off(LED2);
	}
	
	if(!B3){
		 gd_eval_led_on(LED3);
		if(!Sensor2&&!B3_i)
	 {
		 printf("\r\n ��δ�������⵽��� \r\n");
		 rtc_show_time();
		 serial_LCD(3);
	 }
   if(Sensor2&&!B3_i)
	 {
		 serial_LCD(2);//��ӡ������LCD
	 }
	 	 B2_i = 0;
	   B3_i = 1;
 }else
	{
		gd_eval_led_off(LED3);
	}
	
}
//ѡ���������������ʾ�����䣬ͬʱ���ø�ʽ����show�����У�ѡ��ͬ�ĳ��� ���⣬��led2 led3�ֱ�ָʾ��ǰ����ʾ����������
//��ʼ���Ƕο��Բ��ܡ�

void serial_LCD(uint8_t x)//��Ļ��ʾ����
{
	//0��1 --- ���͵紫����
	//2��3 --- ��δ�����
	//4��5 --- ��Ƶ��أ�ͼ�񴫸�����
	
	 if(x == 0)
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t3.txt=");
    putchar(0x22);//˫����
		printf("����"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t3.bco=2016");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 1)
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t3.txt=");
    putchar(0x22);//˫����
		printf("��"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t3.bco=63488");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 2)// 2��3��Ӧ��δ�����
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t4.txt=");
    putchar(0x22);//˫����
		printf("����"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t4.bco=2016");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 3)
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t4.txt=");
    putchar(0x22);//˫����
		printf("����"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t4.bco=63488");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 4)
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t6.txt=");
    putchar(0x22);//˫����
		printf("����"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t6.bco=2016");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 5)
	 { 
		printf("��ʾ�����");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t6.txt=");
    putchar(0x22);//˫����
		printf("����"); 
		putchar(0x22);//˫����
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t6.bco=63488");//��ɫ
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 
}
