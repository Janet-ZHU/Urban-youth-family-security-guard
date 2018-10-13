#ifndef SCCB_H
#define SCCB_H

#include "gd32f3x0.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "GD32-Colibri-F350Rx.h"


typedef unsigned int uint;
typedef unsigned char uchar;

void I2C_Initializes(void);//包括摄像头上电复位
int I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_GetAck(void);
void I2C_SendByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t ack);
uchar SCCBReadByte(uint Addr);
void delay_50ms(void);
void I2C_delay(void);
void init_camera(void);
#endif /* SCCB_H */
