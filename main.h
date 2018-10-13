/*!
    \file  main.h
    \brief the header file of main 
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-06-06, V1.0.0, firmware for GD32F3x0
*/

#ifndef __MAIN_H
#define __MAIN_H

/* led spark function */
//void led_spark(void);
#include "gd32f3x0.h"
#include "systick.h"
#include <stdio.h>
#include "GD32-Colibri-F350Rx.h"
#include "sccb.h"
#include "ov5640.h"




//设备上电后 默认是8Mhz时钟，
void show_system_clk(void);    
void config_system_clk_8M(void);
void config_system_clk_108M(void);


//图像获取
void config_dma(void);
void config_timer(void);
void config_interrupt(void);



#endif


