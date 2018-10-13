#ifndef OV5640_H
#define OV5640_H

#include "sccb.h"

/* Exported types ------------------------------------------------------------*/
/* Image Sizes enumeration */
typedef unsigned char       u8;
typedef unsigned short int  u16;
typedef unsigned  int       u32;
//存储摄像头ID的结构体
typedef struct
{
  uint8_t PIDH;
  uint8_t PIDL;
}OV5640_IDTypeDef;

#define OV5640_SENSOR_PIDH       0x300A
#define OV5640_SENSOR_PIDL       0x300B
#define  Camera_D0           gpio_input_bit_get(GPIOC, GPIO_PIN_6)
#define  Camera_D1           gpio_input_bit_get(GPIOC, GPIO_PIN_7)
#define  Camera_D2           gpio_input_bit_get(GPIOC, GPIO_PIN_8)
#define  Camera_D3           gpio_input_bit_get(GPIOC, GPIO_PIN_9)
#define  Camera_D4           gpio_input_bit_get(GPIOC, GPIO_PIN_10)
#define  Camera_D5           gpio_input_bit_get(GPIOC, GPIO_PIN_11)
#define  Camera_D6           gpio_input_bit_get(GPIOC, GPIO_PIN_12)
#define  Camera_D7           gpio_input_bit_get(GPIOC, GPIO_PIN_13)
#define  Camera_HS           gpio_input_bit_get(GPIOA, GPIO_PIN_6)  //实际上是HREF 野火的摄像头
#define  Camera_VS           gpio_input_bit_get(GPIOA, GPIO_PIN_5)
#define  Camera_PCLK         gpio_input_bit_get(GPIOA, GPIO_PIN_1) //timer1 CH1 --- DMA CH2


void image_GPIO_init(void);
void OV5640_HW_Init(void);
void OV5640_WriteReg(uint16_t Addr, uint8_t Data);
uint8_t OV5640_ReadReg(uint16_t Addr);
uint8_t OV5640_WriteFW(uint8_t *pBuffer ,uint16_t BufferSize);
void OV5640_init_Config(void);
void nops1ms_108M(void);
void OV5640_OutSize_Set(uint16_t offx, uint16_t offy,uint16_t width,uint16_t height);

//战舰
void OV5640_WR_Reg(uint16_t reg, uint8_t data);
uint8_t OV5640_RD_Reg(uint16_t reg);
void OV5640_JPEG_Mode(void);
uint8_t OV5640_Focus_Init(void);
void OV5640_RGB565_Mode(void); 
void OV5640_Light_Mode(u8 mode);
void OV5640_Color_Saturation(u8 sat);
void OV5640_Brightness(u8 bright);
void OV5640_Contrast(u8 contrast);
void OV5640_Sharpness(u8 sharp);
u8 OV5640_Focus_Constant(void);
#endif /* OV5640_H */

