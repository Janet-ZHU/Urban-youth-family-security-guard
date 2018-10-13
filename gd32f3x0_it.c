/*!
    \file  gd32f3x0_it.c
    \brief interrupt service routines
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-06-06, V1.0.0, firmware for GD32F3x0
*/

#include "gd32f3x0_it.h"
#include "systick.h"
#include "ov5640.h"
extern uint8_t flag; //flag图像处理专用
extern uint8_t flag1;
uint8_t flag_i=0;
extern uint8_t  v_switch,h_switch;
extern uint16_t vs_i, hs_i;
extern uint8_t data_buf[1200];
extern uint32_t p_i, p_j;
uint32_t x;

void pclk_delay(uint32_t i)
{
	while(i--);
}

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
/*void SysTick_Handler(void)
{
   
}*/

/*!
    \brief      this function handles external lines 4 to 15 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI4_15_IRQHandler(void)
{
	   //场中断
		if (RESET != exti_interrupt_flag_get(EXTI_5)) 
		{
			vs_i++;
			exti_interrupt_flag_clear(EXTI_5);
			hs_i = 0;			
			if(vs_i == 1)//采集图像，同时将vs_i为0
			{
				exti_interrupt_enable(EXTI_6);
				flag = 0;
			}
			
      if(vs_i == 2)//vs_i =1时的图像已经采集完毕
			{
			  exti_interrupt_disable(EXTI_5);//图像处理完成，函数会打开场中断
				exti_interrupt_disable(EXTI_6);
				vs_i = 0;
				if(flag_i == 0)
				{
					flag1 = 0;
				}
				if(flag_i == 1)
				{
					flag1 = 1;
				}
				if(flag_i == 2)
				{
					flag1 = 2;
				}
				
				flag = 1;//第一帧图像采集完毕，进入图像处理
				
				if(flag_i<2) flag_i++;
			}
			
		}
	
		//行中断
	if (RESET != exti_interrupt_flag_get(EXTI_6)) //取偶数行，一行80个点，一共50行
		{
			exti_interrupt_flag_clear(EXTI_6);//清除标志,必须写在前边
			hs_i++;
			if((hs_i >= 2)&&(hs_i<= 100) && (hs_i%2 == 0))
				dma_channel_enable(DMA_CH2);
			else
				dma_channel_disable(DMA_CH2);
    //偶数行得到是YYYY图像160byte，奇数行是YUYV 320bytes，
	
		}	

 
}


	


