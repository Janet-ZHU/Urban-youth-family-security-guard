#include "ov5640.h"
#include "ov5640af.h"
//OV5640初始化寄存器序列表 
const uint16_t ov5640_init_reg_tbl[][2]= 
{   
	//15fps VGA(640*480) YUV output
// 24MHz input clock,  PCLK
	
	

	0x3008, 0x42, // software power down, bit[6]
	0x3103, 0x03, // system clock from PLL, bit[1]
	0x3017, 0xff, // FREX, Vsync, HREF, PCLK, D[9:6] output enable
	0x3018, 0xff, // D[5:0], GPIO[1:0] output enable
	
  0x3037, 0x13, // PLL root divider, bit[4], PLL pre-divider, bit[3:0]  24/6=4MHz
  0x3036, 0x69,	//	125倍频	 = 500MHz
  0x3035, 0x11, //可以降5倍 0x51
	0x3034, 0x1a, // MIPI 10-bit 默认值
	0x3108, 0x2b, // PCLK root divider, bit[5:4], SCLK2x root divider, bit[3:2]PCLK=pll_CLKI/4;SCLK=PLL_CLKI/8
                //PCLK = 500  /2 /2.5 /4  = 25MHz
	//0x3108, 0x2b, YUV
	// SCLK root divider, bit[1:0]
	//0x4837, 0x22, // MIPI global timing//新增添
	
	0x3630, 0x36,
	0x3631, 0x0e,
	0x3632, 0xe2,
	0x3633, 0x12,
	0x3621, 0xe0,
	0x3704, 0xa0,
	0x3703, 0x5a,
	0x3715, 0x78,
	0x3717, 0x01,
	0x370b, 0x60,
	0x3705, 0x1a,
	0x3905, 0x02,
	0x3906, 0x10,
	0x3901, 0x0a,
	0x3731, 0x12,
	0x3600, 0x08, // VCM control
	0x3601, 0x33, // VCM control
	0x302d, 0x60, // system control
	0x3620, 0x52,
	0x371b, 0x20,
	0x471c, 0x50,
	0x3a13, 0x43, // pre-gain = 1.047x
	0x3a18, 0x00, // gain ceiling
	0x3a19, 0xf8, // gain ceiling = 15.5x
	0x3635, 0x13,
	0x3636, 0x03,
	0x3634, 0x40,
	0x3622, 0x01,
	// 50/60Hz detection 50/60Hz 灯光条纹过滤
	0x3c01, 0x34, // Band auto, bit[7]
	0x3c04, 0x28, // threshold low sum
	0x3c05, 0x98, // threshold high sum
	0x3c06, 0x00, // light meter 1 threshold[15:8]
	0x3c07, 0x08, // light meter 1 threshold[7:0]
	0x3c08, 0x00, // light meter 2 threshold[15:8]
	0x3c09, 0x1c, // light meter 2 threshold[7:0]
	0x3c0a, 0x9c, // sample number[15:8]
	0x3c0b, 0x40, // sample number[7:0]
	0x3810, 0x00, // Timing Hoffset[11:8]
	0x3811, 0x10, // Timing Hoffset[7:0]
	0x3812, 0x00, // Timing Voffset[10:8]
	0x3708, 0x64,
	0x4001, 0x02, // BLC start from line 2
	0x4005, 0x1a, // BLC always update
	0x3000, 0x00, // enable blocks
	0x3004, 0xff, // enable clocks
	0x300e, 0x58, // MIPI power down, DVP enable
	
	0x302e, 0x00,
	//0x4300, 0x30, // YUV 422, YUYV
	0x4300, 0x40, // YUV 420, yyyy/yuyv
	0x501f, 0x00, // YUV 422


	0x440e, 0x00,
	0x5000, 0xa7, // Lenc on, raw gamma on, BPC on, WPC on, CIP on
	// AEC target 自动曝光控制
	0x3a0f, 0x30, // stable range in high
	0x3a10, 0x28, // stable range in low
	0x3a1b, 0x30, // stable range out high
	0x3a1e, 0x26, // stable range out low
	0x3a11, 0x60, // fast zone high
	0x3a1f, 0x14, // fast zone low
	// Lens correction for ? 镜头补偿
	0x5800, 0x23,
	0x5801, 0x14,
	0x5802, 0x0f,
	0x5803, 0x0f,
	0x5804, 0x12,
	0x5805, 0x26,
	0x5806, 0x0c,
	0x5807, 0x08,
	0x5808, 0x05,
	0x5809, 0x05,
	0x580a, 0x08,

	0x580b, 0x0d,
	0x580c, 0x08,
	0x580d, 0x03,
	0x580e, 0x00,
	0x580f, 0x00,
	0x5810, 0x03,
	0x5811, 0x09,
	0x5812, 0x07,
	0x5813, 0x03,
	0x5814, 0x00,
	0x5815, 0x01,
	0x5816, 0x03,
	0x5817, 0x08,
	0x5818, 0x0d,
	0x5819, 0x08,
	0x581a, 0x05,
	0x581b, 0x06,
	0x581c, 0x08,
	0x581d, 0x0e,
	0x581e, 0x29,
	0x581f, 0x17,
	0x5820, 0x11,
	0x5821, 0x11,
	0x5822, 0x15,
	0x5823, 0x28,
	0x5824, 0x46,
	0x5825, 0x26,
	0x5826, 0x08,
	0x5827, 0x26,
	0x5828, 0x64,
	0x5829, 0x26,
	0x582a, 0x24,
	0x582b, 0x22,
	0x582c, 0x24,
	0x582d, 0x24,
	0x582e, 0x06,
	0x582f, 0x22,
	0x5830, 0x40,
	0x5831, 0x42,
	0x5832, 0x24,
	0x5833, 0x26,
	0x5834, 0x24,
	0x5835, 0x22,
	0x5836, 0x22,
	0x5837, 0x26,
	0x5838, 0x44,
	0x5839, 0x24,
	0x583a, 0x26,
	0x583b, 0x28,
	0x583c, 0x42,
	0x583d, 0xce, // lenc BR offset
	// AWB 自动白平衡
	0x5180, 0xff, // AWB B block
	0x5181, 0xf2, // AWB control
	0x5182, 0x00, // [7:4] max local counter, [3:0] max fast counter
	0x5183, 0x14, // AWB advanced
	0x5184, 0x25,
	0x5185, 0x24,
	0x5186, 0x09,
	0x5187, 0x09,
	0x5188, 0x09,
	0x5189, 0x75,
	0x518a, 0x54,
	0x518b, 0xe0,
	0x518c, 0xb2,
	0x518d, 0x42,
	0x518e, 0x3d,
	0x518f, 0x56,
	0x5190, 0x46,
	0x5191, 0xf8, // AWB top limit
	0x5192, 0x04, // AWB bottom limit
	0x5193, 0x70, // red limit
	0x5194, 0xf0, // green limit
	0x5195, 0xf0, // blue limit
	0x5196, 0x03, // AWB control
	0x5197, 0x01, // local limit
	0x5198, 0x04,
	0x5199, 0x12,
	0x519a, 0x04,
	0x519b, 0x00,
	0x519c, 0x06,
	0x519d, 0x82,
	0x519e, 0x38, // AWB control
	// Gamma 伽玛曲线
	0x5480, 0x01, // Gamma bias plus on, bit[0]
	0x5481, 0x08,
	0x5482, 0x14,
	0x5483, 0x28,
	0x5484, 0x51,
	0x5485, 0x65,
	0x5486, 0x71,
	0x5487, 0x7d,
	0x5488, 0x87,
	0x5489, 0x91,
	0x548a, 0x9a,
	0x548b, 0xaa,
	0x548c, 0xb8,
	0x548d, 0xcd,
	0x548e, 0xdd,
	0x548f, 0xea,
	0x5490, 0x1d,
	// color matrix 色彩矩阵	
	0x5381, 0x1e, // CMX1 for Y
	0x5382, 0x5b, // CMX2 for Y
	0x5383, 0x08, // CMX3 for Y
	0x5384, 0x0a, // CMX4 for U
	0x5385, 0x7e, // CMX5 for U
	0x5386, 0x88, // CMX6 for U
	0x5387, 0x7c, // CMX7 for V
	0x5388, 0x6c, // CMX8 for V
	0x5389, 0x10, // CMX9 for V
	0x538a, 0x01, // sign[9]
	0x538b, 0x98, // sign[8:1]
	// UV adjust UV 色彩饱和度调整
	0x5580, 0x06, // saturation on, bit[1]
	0x5583, 0x40,
	0x5584, 0x10, 
	0x5589, 0x10,
	0x558a, 0x00,
	0x558b, 0xf8,
	0x501d, 0x40, // enable manual offset of contrast
	// CIP 锐化和降噪
	0x5300, 0x08, // CIP sharpen MT threshold 1
	0x5301, 0x30, // CIP sharpen MT threshold 2
	0x5302, 0x10, // CIP sharpen MT offset 1
	0x5303, 0x00, // CIP sharpen MT offset 2
	0x5304, 0x08, // CIP DNS threshold 1
	0x5305, 0x30, // CIP DNS threshold 2
	0x5306, 0x08, // CIP DNS offset 1
	0x5307, 0x16, // CIP DNS offset 2
	0x5309, 0x08, // CIP sharpen TH threshold 1
	0x530a, 0x30, // CIP sharpen TH threshold 2
	0x530b, 0x04, // CIP sharpen TH offset 1
	0x530c, 0x06, // CIP sharpen TH offset 2
	0x5025, 0x00, 
	0x3008, 0x02, // wake up from standby, bit[6]
	

	//自行添加的设置
	0x4740, 0X2f, //VSYNC HREF 高有效 //pclk在需要的时候开
	
	0x5001, 0xA3, // SDE on, scaling on, CMX on, AWB on 
  0x3503, 0x00, // AEC/AGC on 打开自动曝光

  //0x503d, 0x80,//测试彩条
 // 0x4741, 0x00,
};  


void image_GPIO_init(void)
{
	  /** PC0~PC7 D0~D7 ; PA12-HS; PA9-VS; PA10--PCLK8其它接口在SCCB中定义*/
	  gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_6); 
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_7);
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_8); 
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_9);
	  gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_10); 
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_11);
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_12); 
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_13);
	  //gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_ALL);
	
	  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_6);//HS
		gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,GPIO_PIN_5); //VS
    
	  //gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	  rcu_periph_clock_enable(RCU_GPIOC); // must set
	  rcu_periph_clock_enable(RCU_GPIOA); // must set
	 
}

//初始化摄像头
void OV5640_HW_Init(void)
{
	
	OV5640_init_Config();
	image_GPIO_init();
}


/**
  * @brief  读取摄像头的ID.
  * @param  OV5640ID: 存储ID的结构体
  * @retval None
  */
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID)
{

	/*读取寄存芯片ID*/
  OV5640ID->PIDH = OV5640_ReadReg(OV5640_SENSOR_PIDH);
  OV5640ID->PIDL = OV5640_ReadReg(OV5640_SENSOR_PIDL);
}

/**
  * @brief  写一字节数据到OV5640寄存器
  * @param  Addr: OV5640 的寄存器地址
  * @param  Data: 要写入的数据
  * @retval none
  */
void OV5640_WriteReg(uint16_t Addr, uint8_t Data)//测试过没有问题
{
	I2C_Start();
	I2C_SendByte(0x78);
	
	I2C_GetAck();//不必在乎是否应答
	I2C_SendByte((uint8_t)((Addr>>8) & 0xFF) );
	I2C_GetAck();//不必在乎是否应答
  I2C_SendByte( (uint8_t)(Addr & 0xFF) );
	I2C_GetAck();//不必在乎是否应答
	

	I2C_SendByte(Data);
	I2C_GetAck();//不必在乎是否应答
	
  I2C_Stop();
}
/**
  * @brief  从OV5640寄存器中读取一个字节的数据
  * @param  Addr: 寄存器地址
  * @retval 返回读取得的数据
  */
uint8_t OV5640_ReadReg(uint16_t Addr)//已经测试过
{
  return SCCBReadByte(Addr);
}

/**
* @brief  Configures the OV5640 camera in JPEG mode.
* @param  JPEGImageSize: JPEG image size
* @retval None
*/


void nops1ms_108M(void) //1ms
{
    uint32_t t=540000;   
	  while(t--);
}

void OV5640_init_Config(void)
{
  uint32_t i;
	I2C_Initializes();//初始化摄像头，I2C，sccb.h
	for(i=0;i<10;i++)
	 {nops1ms_108M();}//delay10ms
	I2C_Initializes();//初始化摄像头，I2C，sccb.h
	 OV5640_WriteReg(0x3103,0X11);
	 OV5640_WriteReg(0X3008,0X82);//软复位
	 	for(i=0;i<10;i++)
	 {nops1ms_108M();}//delay10ms
	 
    /* Initialize OV5640 */
	for(i=0;i<sizeof(ov5640_init_reg_tbl)/4;i++)
	{
	   	OV5640_WR_Reg(ov5640_init_reg_tbl[i][0],ov5640_init_reg_tbl[i][1]);
 	}  
   nops1ms_108M();
          
}
//设置图像输出大小
//OV5640输出图像的大小(分辨率),完全由该函数确定
//offx,offy,为输出图像在OV5640_ImageWin_Set设定窗口(假设长宽为xsize和ysize)上的偏移
//由于开启了scale功能,用于输出的图像窗口为:xsize-2*offx,ysize-2*offy
//width,height:实际输出图像的宽度和高度
//实际输出(width,height),是在xsize-2*offx,ysize-2*offy的基础上进行缩放处理.
//一般设置offx和offy的值为16和4,更小也是可以,不过默认是16和4 
//返回值:0,设置成功
//    其他,设置失败
void OV5640_OutSize_Set(uint16_t offx, uint16_t offy,uint16_t width,uint16_t height)
{ 
    OV5640_WriteReg(0X3212,0X03);  	//开始组3
    //以下设置决定实际输出尺寸(带缩放)
    OV5640_WriteReg(0x3808,width>>8);	//设置实际输出宽度高字节
    OV5640_WriteReg(0x3809,width&0xff);//设置实际输出宽度低字节  
    OV5640_WriteReg(0x380a,height>>8);//设置实际输出高度高字节
    OV5640_WriteReg(0x380b,height&0xff);//设置实际输出高度低字节
	//以下设置决定输出尺寸在ISP上面的取图范围
	//范围:xsize-2*offx,ysize-2*offy
    OV5640_WriteReg(0x3810,offx>>8);	//设置X offset高字节
    OV5640_WriteReg(0x3811,offx&0xff);//设置X offset低字节
	
    OV5640_WriteReg(0x3812,offy>>8);	//设置Y offset高字节
    OV5640_WriteReg(0x3813,offy&0xff);//设置Y offset低字节
	
    OV5640_WriteReg(0X3212,0X13);		//结束组3
    OV5640_WriteReg(0X3212,0Xa3);		//启用组3设置

}
void OV5640_WR_Reg(uint16_t reg,uint8_t data)
{
	OV5640_WriteReg(reg,data);
}
 uint8_t OV5640_RD_Reg(uint16_t  reg)
{
	return SCCBReadByte(reg);
}


//测试序列
//mode:0,关闭
//     1,彩条 
//     2,色块
void OV5640_Test_Pattern(u8 mode)
{
	if(mode==0)OV5640_WR_Reg(0X503D,0X00);
	else if(mode==1)OV5640_WR_Reg(0X503D,0X80);
	else if(mode==2)OV5640_WR_Reg(0X503D,0X82);
} 


void delay_ms(u8 t)
{
	  u8 i;
		for(i=0;i<t;i++)
	 {nops1ms_108M();}//delay10ms
}

