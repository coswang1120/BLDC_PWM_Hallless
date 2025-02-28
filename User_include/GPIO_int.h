//############################################################
// FILE: GPIO_int.h
// Created on: 2017年1月18日
// Author: XQ
// summary: GPIO_int
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef _GPIO_int_H
#define _GPIO_int_H
#include "stm32f10x.h"
 
//带参宏，可以像内联函数一样使用
#define LED_485RE_ON   	GPIO_SetBits(GPIOB,GPIO_Pin_2) 	 
#define LED_485RE_OFF   GPIO_ResetBits(GPIOB,GPIO_Pin_2) 
 
void Delay(u32 nCount); 
void Init_Gpio_ADC(void);
void GPIO_LED485RE_int(void);	
void Init_Gpio_TIM1_PWM(void);
void InitThree_BEF_Gpio(void); 
void InitCAN_Gpio(void );
void InitUSART3_Gpio(void );
void Init_PWMDAC_Gpio (void);
void LED1_Toggle(void); 
#endif   //  GPIO_int.h
//===========================================================================
// No more.
//===========================================================================
