//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017��1��15��
// Author: XQ
// summary: Tim1_ISR_MCLoop
// ��ʱ��1������ƣ� �жϻ�·�ջ�����  
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//��Ȩ���У�����ؾ�
//DSP/STM32������ƿ�����
//˶������
//��ַ: https://shuolidianzi.taobao.com
//�޸�����:2017/9/8
//�汾��V17_3
//Author-QQ: 616264123
//�������QQȺ��314306105
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
// һ�����ֵ�ͨ�˲���   328+696=1024  a=328/1024
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search

void TIM1_UP_IRQHandler(void) // ����ADC�жϲ����͵����·����
{
    //�˳�������ж�һֱִ��

    // ����ĸ�ߵ�����ѹ�ɼ�  PWM�жϽ����ȶ�ȡAD������λ�û���
    ADC_Sample(); // AD������ĸ�ߵ�ѹ�͵�������λ������
    BEFHall_huanxkz(); // ���ݷ��綯��״̬����


    if (logicContr.drive_car == 0) //
    {
        Offset_CurrentReading(); // ��������ƫִ��ѹ��ȡ
    }

    TaskTimePare.PWMZD_count++;
    if (TaskTimePare.PWMZD_count > 24) //����24*83.333us=2msʱ������  12K����Ƶ��
    {
        TaskTimePare.PWMZD_count = 0;

        VF_start_control(); //�޸�������VF�̶�����
        knob_control(); //catch variable resist for rotational speed
        //ͨѶ�������Դ��

        TestPare.fact_BUS_Voil = ADCSampPare.BUS_Voltage;
        TestPare.fact_BUS_Curr = ADCSampPare.BUS_CurrF;
        TestPare.Speed_fact = pi_spd.Fbk;
        TestPare.Speed_target = pi_spd.Ref;

        if (logicContr.Start_order == 0) //�رղ���
        {
            BEF_ThreeCAP.Speed_RPM = 0;
            BEF_ThreeCAP.Speed_RPMF = 0;
            pi_spd.Fbk = 0;
        }

        if (logicContr.Start_order == 3) //��������3 �л��޸з��綯���ٶȱջ�����
        {
            pi_spd.Fbk = BEF_ThreeCAP.Speed_RPMF; //   0--4096
            PI_Controller((p_PI_Control) &pi_spd); // �ٶȻ�PID
            pi_spd.OutF = _IQ10mpy(FilK1, pi_spd.OutF) + _IQ10mpy(FilK2, pi_spd.Out);
            DUTY = pi_spd.OutF; // �ջ�
        }
    }

    Protection_software(); // �������ֱ�ӹرչ���  600��3A���ң��ɵ����޸ģ�Ӳ����׼�����3A

    //��ʱ��4�ڳ�����������������ͬ�����FOC����������
    //���������������PWM���������ͨ��RC�����ͨ�˲���Ϳ��Բ⵽���ݲ��Σ� pwmռ�ձȲ���
    TIM_SetCompare3(TIM4, BEF_ThreeCAP.Speed_RPMF); // DACͨ��1
    TIM_SetCompare4(TIM4, DUTY); // DACͨ��2

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}


//===========================================================================
// No more.
//===========================================================================
