#include "main.h"

//个人应用程序，主要在main.c 和 gd32f3x0_it.c文档末尾的ETI中断处理函数中定义，依赖于GD32F350官方固件库，
//本程序使用了GPIO，TIMER捕获，RTC，DMA，外部中断，I2C，FLASH, UART，PLL等模块。
//摄像头驱动程序，SCCB/ov5640。
//本程序仅供学习使用，EEworld ID:传媒学子, 可通过EEworld论坛给我留言，讨论。
//本程序，适用于兆易创新公司生产的GD32F350 arm处理器。
//最后，感谢兆易创新公司和EEworld论坛能够举办这次创新大赛！


//以下函数只有mian.c中用到，其它函数在main.h中定义，方便其它.c文件引用
void rtc_setup(void);
void rtc_show_time(void);
void rtc_pre_config(void);
void image_process(void);//图像处理
void serial_LCD(uint8_t x);
void FLASH_ProgramBytes(uint32_t Address, uint8_t *Buffer, uint16_t ByteCount);
void rain_and_rsd_senser(void);

// 程序中LED1 对应  PB10 对应板子上LED4最上边， LED2  PB8  LED5中间， LED3 最下边的呢一个PB9

//实时时钟定义
#define RTC_CLOCK_SOURCE_IRC40K 
#define BKP_VALUE    0x32F0
rtc_parameter_struct   rtc_initpara;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;


uint8_t flag=0;//flag =1,进入图像处理，flag = 0,不进行处理	
uint8_t flag1=0;//第一次数据，data1和data2同时存取，之后data1存取新的，data2存取data1的	
uint8_t flag2=0;//屏蔽开机阶段的几个的图像判断，防止开机误显示

#define OV5640_DEVICE_ADDRESS    0x78  // 0111 1000 ov5640 I2C设备地址，读的时候0x78，写的时候0x79
//uint8_t v_switch = 1,h_switch = 0;
uint16_t vs_i=0,hs_i=0; //begin 开始采集图像
uint32_t p_i = 0, p_j = 0;
__IO uint16_t data_buf[4050]={0};	//YUV 4;2;2  YUYV  每个Y加一个色差信号为一个像素，因此160*120应该输出 38400个byte;
uint8_t image_buf[4050]={0};	//输出81*50的8-bit灰度图像;

uint8_t data0[450] ={0};
uint8_t data1[450] ={0};//存储当前帧图像
uint8_t data2[450] ={0};//存储上帧图像
const uint32_t data3[1013] ={0};//出现告警时，存储告警图像至flash中data3中 //4050/4 = 1013


//定义DMA和Timer初始化结构体
dma_parameter_struct DMA_InitStructure;
timer_parameter_struct TIM_TimeBaseStructure;
timer_ic_parameter_struct	TIM_ICInitStructure;


#define B2  gpio_input_bit_get(GPIOA, GPIO_PIN_0)//按键
#define B3  gpio_input_bit_get(GPIOB, GPIO_PIN_7) //按键
uint8_t B2_i = 0,B3_i =0; //按键显示控制

#define Sensor1  gpio_input_bit_get(GPIOC, GPIO_PIN_0)//热释电
#define Sensor2  gpio_input_bit_get(GPIOC, GPIO_PIN_1) //雨滴


//图像处理算法，固定摄像头后，当有人出现时，会告警
void image_process(void)
{
	 uint16_t  i, j = 0, k = 0, dot_diff=0;
	 //exti_interrupt_disable(EXTI_5);	 flag == 1 时已经关闭场中断
   uint32_t image_diff = 0;
	 for( i=0;i<4050;i++){	 //将取出的图像进行整理
			 image_buf[i] = (uint8_t)(data_buf[i]>>6);
			 j++;
			 if(j == 9) //每行取9个数据点
			 {
				j=0;
				data0[k++] = image_buf[i];
			 }	
		 }				 
	
	 if(!flag1){ //flag1 == 0, 同时存取data0数据
		 for( i=0;i<450;i++){	 
				data1[i] = data0[i];  
			  data2[i] = data0[i]; 
		 }
		 //flag1 = 1;
    }
	 else if(flag1 == 1)//flag1 == 1, 只有data1存取data0数据
	 {
			 for( i=0;i<450;i++){	 
			   data1[i] = data0[i];  
			 }
		   //flag1 = 2; 
	 }
	 else{ //flag1 == 2, data2先存取data1数据，同时data1存取data0数据,
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
		if(dot_diff > 20) //判断两幅图像中，不同的点有多少个，此阈值影响图像识别准确度，需根据实际情况优化
		image_diff += 1;
	}
	if(image_diff > 40) //两幅图像中不同点数，超过此阈值，视为有人闯入，需根据实际情况优化
	{
		if(flag2<=2) //屏蔽前两次判断
		{
			flag2++;
			serial_LCD(0);//显示正常
			serial_LCD(2);
			serial_LCD(4);
		}
		if(flag2 == 3)
		{
		printf("\r\n 告警！有人闯入!  "); 
		rtc_show_time();
		serial_LCD(5);//显示告警
//		
//		for(i=0; i<4050;i++)
//		{
//			FLASH_ProgramBytes(*data3, image_buf, 4050);//存储告警图像
//			//添加其它处理
//		}
		}
	}
	else 
	{
		//printf("\r\n 正常 \r\n");
		//serial_LCD(4);
	}
   exti_interrupt_enable(EXTI_5);	//处理完毕，使能场中断
}



void nops500ms_108M(void) //1ms
{
    uint32_t t=54000000;   
	  while(t--);
}
//USART0对应PA2,PA3

int main(void)
{
	  rcu_periph_clock_enable(RCU_GPIOA);//需要开启，串口才能使用
		gd_eval_com_init(EVAL_COM1,115200);
	  //蓝牙串口软件暂时只能显示英文，下面这段文字是测试英文使用
	  printf(" Welcome you ! Now this is buletooth connection.\n");
    printf(" Start the moniter mode... \n");
    printf(" Next, transfer to PC using 115200, buletooth will waiting... \n");
	
	 //下面这些是调试时使用
	
	  printf("\r\n***都市青年家庭安防卫士***");
		printf("\r\n");
	  printf("\r\n       Welcome you!   \r\n");
		
		rtc_setup();//配置RTC实时时钟
	  show_system_clk();//显示当前时钟
    rcu_periph_clock_enable(RCU_CFGCMP);//need to be started
				
		OV5640_HW_Init();//初始化摄像头，YUV420输出，YYYY/YUYV 第一行输出YYYYYY 第二行输出YUYV 两行合起来4：2：0
    OV5640_OutSize_Set(0,0,160,120);//设置输出尺寸 
		
		gd_eval_led_init(LED1);
		gd_eval_led_init(LED2);
		gd_eval_led_init(LED3);

		gd_eval_led_off(LED1);
		gd_eval_led_off(LED2);
		gd_eval_led_off(LED3);
		
		gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);//初始化按键B2，作为判断是否主动开启雨滴传感器的标志对应LED2
		gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);//初始化按键B3，作为判断是否主动开启热释电传感器的标志对应LED3
		
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_0);//热释电 输出1代表感应到人体，该模块有8s左右的延迟
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_1);//雨滴传感器，输出0 代表有雨滴


		printf("\r\n各项传感器状态：");
		printf("\r\n***热释电传感器配置完成***");
		printf("\r\n***蓝牙通信模块配置完成***");
	  printf("\r\n***雨滴传感器模块配置完成***");
		
		if((0x56 == SCCBReadByte(0x300A)) && (0x40 == SCCBReadByte(0x300B)))
    {
			printf("\r\n***摄像头已启动***");
		  gd_eval_led_on(LED1);//LED1，亮，摄像头初始化完成
		}
		else
	  printf("\r\n 未检测到摄像头，或者摄像头异常！");
		
		printf("\r\n");
    printf("\r\n***系统自检完成，进入监测模式***");
    printf("\r\n");
		rtc_show_time();
		
		//图像场中断、行中断分别由PA5 PA6 作为外部中断输入，由定时器捕捉PCLK(Timer1_CH1),触发DMA从外设GPIOC传输数据到内存中
		// 定时器捕获和DMA 合作，完成
    config_dma();//初始化DMA
	  config_timer();//初始化timer
	  config_interrupt();//初始化中断
		
	 
		
    while (1){
			   if(flag)//图像处理
				 {
           image_process();
					 flag = 0;
				 }
				 rain_and_rsd_senser();//雨滴传感器处理
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
    rcu_periph_clock_enable(RCU_GPIOA); //在ov5640初始化中已经配置了
	  rcu_periph_clock_enable(RCU_CFGCMP);//在主函数开头配置
    /* configure button pin as input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_5); //vs
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6); //HREF
 
	
		/* enable and set EXTI interrupt to the lowest priority */
    nvic_irq_enable(EXTI4_15_IRQn, 2U, 0U);
		/* connect key EXTI line to key GPIO pin */
		syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN5);
	  syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN6);



		/* configure EXTI line */
	  exti_init(EXTI_5, EXTI_INTERRUPT, EXTI_TRIG_RISING);//上升沿
		exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_RISING);//上升沿
  // exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_RISING);//上升沿
	
		exti_interrupt_flag_clear(EXTI_5);
	  exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_disable(EXTI_6);//先关闭行中断 行中断为12
	
}

void config_dma(void)
{
	 rcu_periph_clock_enable(RCU_DMA);
	 DMA_InitStructure.periph_addr  = (uint32_t) &GPIO_ISTAT(GPIOC);//不确定需不需要加&
	 DMA_InitStructure.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	 DMA_InitStructure.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	 DMA_InitStructure.memory_addr = (uint32_t) data_buf;//不确定需不需要加&
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
	 //PA1复用
	 gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_1);//PCLK
	 gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_1);
	//TIM1
	rcu_periph_clock_enable(RCU_TIMER1);
	TIM_TimeBaseStructure.prescaler = 0;
	TIM_TimeBaseStructure.alignedmode = TIMER_COUNTER_EDGE;
	TIM_TimeBaseStructure.counterdirection = TIMER_COUNTER_UP;
	TIM_TimeBaseStructure.period = 2;// 从0~200
	TIM_TimeBaseStructure.clockdivision = TIMER_CKDIV_DIV1;
	TIM_TimeBaseStructure.repetitioncounter = 0;
	timer_init(TIMER1, &TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.icpolarity = TIMER_IC_POLARITY_RISING;
	TIM_ICInitStructure.icselection = TIMER_IC_SELECTION_DIRECTTI;
	TIM_ICInitStructure.icprescaler = TIMER_IC_PSC_DIV2;//摄像头PCLK2分频，一行取80个点
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
			  printf("  Today: 20%0.2x年%0.2x月%0.2x日 \n\r", \
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




//将系统时钟源切换到8MHz
void config_system_clk_8M(void)//自有函数
{
		//rcu_deinit();//不要初始化RCU，会造成问题
		RCU_CFG0 &= 0xFFFFFFFC;//将SCS置位为00
		while(RCU_CFG0&0x0000000C);//判断SCSS为00，转换完成
		//gd_eval_com_init(EVAL_COM1);
		printf("\r\nRCU_CFG0：%x",RCU_CFG0);
		show_system_clk();
}

void config_system_clk_108M(void)//自有函数
{
	  printf("\r\n等待pll重启..");
	  RCU_CTL0 &= ~BIT(24); //关闭PLLEN
		RCU_CTL0 &= ~BIT(25); //将PLL稳定标志清零
		RCU_CFG0 &= ~BIT(19);//BIT19 变换倍频为20
		RCU_CTL0 |= BIT(24); //使能PLLEN
		while(!(RCU_CTL0&(BIT(25))));
		RCU_CFG0 |= BIT(1);//将SCS置位为10
		while(!(RCU_CFG0&0x0000000C));//判断SCSS为10，转换完成
		//gd_eval_com_init(EVAL_COM1);
		show_system_clk();
}

void show_system_clk(void)//自建函数
{
	 if(8 == rcu_system_clock_source_get())
	 printf("\r\n目前的系统时钟源为：CK_PLL");//10 00 - CK_PLL;0000 CK_IRC8M
   printf("\r\n目前的系统时钟频率为：%dHz",rcu_clock_freq_get(CK_SYS));
   printf("\r\n目前的AHB时钟频率为：%dHz",rcu_clock_freq_get(CK_AHB));
	 printf("\r\n目前的CK_APB1时钟频率为：%dHz",rcu_clock_freq_get(CK_APB1));
	 printf("\r\n目前的CK_APB2时钟频率为：%dHz",rcu_clock_freq_get(CK_APB2));
	 printf("\r\n目前的CK_ADC时钟频率为：%dHz",rcu_clock_freq_get(CK_ADC)); //28M/2
	 printf("\r\n目前的CK_CEC时钟频率为：%dHz",rcu_clock_freq_get(CK_CEC));
	 printf("\r\n目前的CK_USART时钟频率为：%dHz  ",rcu_clock_freq_get(CK_USART));
}


void FLASH_ProgramBytes(uint32_t Address, uint8_t *Buffer, uint16_t ByteCount)		//Flash写入多个8位字节
{
    uint16_t i = 0;														//循环次数
    fmc_unlock();;
    while(i<ByteCount)			//循环字节次数
    {
	   fmc_word_program(Address, *(u32*)Buffer);
        i = i+4;													//字节数+4
        Address = Address + 4;										//地址+4
        Buffer = Buffer + 4;										//要写入的数据字节+4
			  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    }
    fmc_lock();
}


//************************************************************************************
//***这里只是demo,在实际应用当中，可以降这两个传感器监测放入中断中处理，这样更好
//************************************************************************************
void rain_and_rsd_senser(void)//雨滴传感器和热释电传感器
{
	
	//按键主动监测
	if(!B2){ //为调试方便按下B2才检测热释电
	 gd_eval_led_on(LED2);
   if(Sensor1&&!B2_i)//因为，调试附近有人，所以这个传感器一直为有效状态
	 {	 
		 printf("\r\n 热释电传感器探测到人\r\n");
		 rtc_show_time();
		 serial_LCD(1);
	 }
	 if(!Sensor1&&!B2_i)
	 {
		 serial_LCD(0);//打印正常至LCD
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
		 printf("\r\n 雨滴传感器监测到雨滴 \r\n");
		 rtc_show_time();
		 serial_LCD(3);
	 }
   if(Sensor2&&!B3_i)
	 {
		 serial_LCD(2);//打印正常至LCD
	 }
	 	 B2_i = 0;
	   B3_i = 1;
 }else
	{
		gd_eval_led_off(LED3);
	}
	
}
//选择蓝牙传输或者显示屏传输，同时设置格式，在show函数中，选择不同的程序； 另外，用led2 led3分别指示当前是显示屏还是蓝牙
//开始的那段可以不管。

void serial_LCD(uint8_t x)//屏幕显示控制
{
	//0，1 --- 热释电传感器
	//2，3 --- 雨滴传感器
	//4，5 --- 视频监控（图像传感器）
	
	 if(x == 0)
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t3.txt=");
    putchar(0x22);//双引号
		printf("正常"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t3.bco=2016");//绿色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 1)
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t3.txt=");
    putchar(0x22);//双引号
		printf("是"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t3.bco=63488");//红色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 2)// 2，3对应雨滴传感器
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t4.txt=");
    putchar(0x22);//双引号
		printf("正常"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t4.bco=2016");//绿色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 3)
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t4.txt=");
    putchar(0x22);//双引号
		printf("下雨"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t4.bco=63488");//红色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 4)
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t6.txt=");
    putchar(0x22);//双引号
		printf("正常"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t6.bco=2016");//绿色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 	if(x == 5)
	 { 
		printf("显示屏命令：");
		putchar(0xff);putchar(0xff);putchar(0xff);
	  printf("t6.txt=");
    putchar(0x22);//双引号
		printf("有人"); 
		putchar(0x22);//双引号
    putchar(0xff);putchar(0xff);putchar(0xff);		 
		printf("t6.bco=63488");//红色
	  putchar(0xff);putchar(0xff);putchar(0xff);
		printf("\n");
	 }
	 
}
