//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017年1月15日
// Author: XQ
// summary: Tim1_ISR_MCLoop
// 定时器1电机控制， 中断环路闭环控制  
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2017/9/8
//版本：V17_3
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "Tim1_ISR_MCLoop.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
#include "GPIO_int.h"
#include "BEF_Hall.h"
#include "VF_angle.h"
#include "Task_function.h"
#include "PI_Cale.h"
#include "Timer.h"
//#include "Usart_RS232.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "printf_uart.h"




extern PI_Control pi_spd;
extern ADCSamp ADCSampPare;
extern uint16_t PWM_DUTY[3];
extern logic logicContr;
extern TaskTime TaskTimePare;
extern Test TestPare;
extern Hall BEF_ThreeCAP;

extern uint16_t DUTY;

uint16_t FilK1 = 328;
uint16_t FilK2 = 696;
// 一阶数字低通滤波器   328+696=1024  a=328/1024
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search

void TIM1_UP_IRQHandler(void) // 触发ADC中断采样和电机环路控制
{
    //此程序更新中断一直执行

    // 电流母线电流电压采集  PWM中断进入先读取AD电流和位置换向
    ADC_Sample(); // AD采样，母线电压和电流，电位器采样
    BEFHall_huanxkz(); // 根据反电动势状态换相


    if (logicContr.drive_car == 0) //
    {
        Offset_CurrentReading(); // 电流采样偏执电压读取
    }

    TaskTimePare.PWMZD_count++;
    if (TaskTimePare.PWMZD_count > 24) //计算24*83.333us=2ms时间任务  12K开关频率
    {
        TaskTimePare.PWMZD_count = 0;

        VF_start_control(); //无感启动的VF固定启动
        knob_control(); //catch variable resist for rotational speed
        //通讯参数测试打包

        TestPare.fact_BUS_Voil = ADCSampPare.BUS_Voltage;
        TestPare.fact_BUS_Curr = ADCSampPare.BUS_CurrF;
        TestPare.Speed_fact = pi_spd.Fbk;
        TestPare.Speed_target = pi_spd.Ref;

        if (logicContr.Start_order == 0) //关闭步骤
        {
            BEF_ThreeCAP.Speed_RPM = 0;
            BEF_ThreeCAP.Speed_RPMF = 0;
            pi_spd.Fbk = 0;
        }

        if (logicContr.Start_order == 3) //启动步骤3 切换无感反电动势速度闭环控制
        {
            pi_spd.Fbk = BEF_ThreeCAP.Speed_RPMF; //   0--4096
            PI_Controller((p_PI_Control) &pi_spd); // 速度换PID
            pi_spd.OutF = _IQ10mpy(FilK1, pi_spd.OutF) + _IQ10mpy(FilK2, pi_spd.Out);
            DUTY = pi_spd.OutF; // 闭环
        }
    }

    Protection_software(); // 软件保护直接关闭管子  600是3A左右，可调试修改，硬件不准许大于3A

    //定时器4在除正交编码器的永磁同步电机FOC控制中运用
    //其他程序代码做用PWM输出数字量通过RC电阻低通滤波后就可以测到数据波形， pwm占空比波形
    TIM_SetCompare3(TIM4, BEF_ThreeCAP.Speed_RPMF); // DAC通道1
    TIM_SetCompare4(TIM4, DUTY); // DAC通道2

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}


//===========================================================================
// No more.
//===========================================================================
