//############################################################
// FILE:  main.c
// Created on: 2017年1月15日
// Author: XQ
// summary: main 主程序的MCU底层初始化，通讯函数
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2018/7/12
//版本：V17_3
//Author-QQ: 616264123
//电机控制QQ群：314306105

//  hall sensor

//  4 poles pair



// TIM1   -->  PWM (真正PWM)    TIM4  -->  DEMO PWM  可以拉出來看訊號 [TIM4PWMDAC_Config]

//  sample time=83us   control time=2ms
//  Hall_Three.Speed_RPMF   estimated rpm from hall sensor
//  pi_spd.Ref   rotation button  0 -4000   [target speed]

//  Duty   max-> 4000

//  can use time.c add parameter to debug




/* GPIOA,GPIOB, Configuration: PWM
    GPIOA 8    9      10     PMWUH
    GPIOB 13  14    15  PMWUL
*/



//############################################################


#include "stm32f10x.h"
#include "GPIO_int.h"
#include "Timer.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
//#include "ThreeHall.h"
#include "BEF_Hall.h"
#include "IQ_math.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "VF_angle.h"
#include "PI_Cale.h"
#include "Task_function.h"
//#include "Usart_RS232.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//#include <math.h>
#include "printf_uart.h"
#include "main.h"
//#include "hello_world.h"
//#include <stm32f10x_usart.h>
#include <string.h>



u16 t;
u16 len;
long numdec=0;
float syst=0.0;
PI_Control   pi_spd = PI_Control_DEFAULTS;
//PI_Control   pi_ICurr = PI_Control_DEFAULTS;
Test         TestPare=Test_DEFAULTS;
TaskTime     TaskTimePare=TaskTime_DEFAULTS;
logic        logicContr=logic_DEFAULTS;
ADCSamp      ADCSampPare=ADCSamp_DEFAULTS;
VF_start     VF_startPare=VF_start_DEFAULTS ;
Hall         BEF_ThreeCAP=Hall_DEFAULTS;

extern u16 Tag;
u16 USART_RX_STA;
u16 spdcmd;
extern  uint16_t  DUTY;
u16 rxcmd;
//u8 switchRX;


char uart_rx_buffer[UART_RX_BUFFER_SIZE]; // 串口接收緩衝區
volatile uint16_t uart_rx_write_ptr = 0; // 寫指針
volatile uint8_t uart_rx_line_complete = 0; // 行結束標誌

#define MOTOR_CMD_1 "Motor set 0"
#define MOTOR_CMD_2 "Motor set 1"
#define MOTOR_CMD_3 "Motor set 2"

// 定義枚舉類型
typedef enum {
    CMD_0,
    CMD_1,
    CMD_2,
    CMD_UNKNOWN
} MotorSpeedCmd;



// 提取處理命令的函數
MotorSpeedCmd getMotorSpeedCmd(const char* cmd) {
    if (strcmp(cmd, MOTOR_CMD_1) == 0) return CMD_0;
    if (strcmp(cmd, MOTOR_CMD_2) == 0) return CMD_1;
    if (strcmp(cmd, MOTOR_CMD_3) == 0) return CMD_2;
    return CMD_UNKNOWN;
}
// 函數定義
void process_uart_command(void) {
    if (uart_rx_line_complete) {
        // 移除結尾的 \r\n
        uart_rx_buffer[uart_rx_write_ptr-2] = '\0';

        switch (getMotorSpeedCmd(uart_rx_buffer)) {
            case CMD_0:
                rxcmd = 0;
            break;
            case CMD_1:
                rxcmd = 1000;
            break;
            case CMD_2:
                rxcmd = 2500;
            break;
            default:
                // 處理未知命的情況
                    rxcmd = 0;
            break;
        }
        // 在這裡處理接收到的命令
        printf("Motor %d rpm OK! \r\n", rxcmd);

        // 重置緩衝區
        uart_rx_write_ptr = 0;
        uart_rx_line_complete = 0;
    }
}

int main(void) {
    // 2ms  control  knob control
    Delay(10000);
    //SysTickConfig();              // 10ms
    Timer2Config(); // 10ms

    //logicContr.Control_Mode = 2;  //   1 -> DUTY = 2*pi_spd.Ref    2 ->   close
    //   loop    I & rotational speed
    logicContr.Run_mode = 1; //     1 ->  CCW     2 -> CW
    //switchRX=0;                   //   轉變外部輸入(只能是 1 -> DUTY = 2*pi_spd.Ref )   0: 電阻器  1:外部RX輸入 (0/1500/2500)


    GPIO_LED485RE_int(); // Blink LED initial
    Init_Gpio_ADC(); // ADC的引脚初始化      83us
    //InitUSART3_Gpio();            // 串口3IO初始化
    Init_Gpio_TIM1_PWM(); // 高级定时器1的6个IO初始化   // pwm
    // 12K       83.333us

    InitThree_BEF_Gpio(); //  反电动势过零比较IO初始化
    BEF_ThreeCAPPara_init(); //反电动势过零比较参数初始化
    VF_start_init(); //开环启动控制
    //InitThreeHallGpio();          // 霍尔的IO初始化
    Init_PWMDAC_Gpio(); // PWM4的IO作为DAC初始化
    //ThreeHallPara_init();         // 三霍尔角度传感器的参数初始化
    //Usart3_RS232_init();          // 串口3初始化
    DMA_Configuration(); // ADC连接DMA读取数据初始化
    Delay(10000);
    ADC1_Configuration(); // ADC模式初始化      1us
    Delay(10000);
    Tim1_PWM_Init(); // 高级定时器1初始化   Tim1 ini   Tim int  ->   ADC /
    // offset curr  /PWM   /   6 step     /    -->
    // TIM1_ISR_MCLoop controller scheme  83 us
    Delay(10000);
    TIM4PWMDAC_Config(); // TIM4的 作为DAC初始化
    Delay(10000);
    Offset_CurrentReading(); // 电机的母线电流采样偏执电压   initial current
    // U_Curr   V_Curr  Bus_Curr
    Delay(10000);
    PI_Pare_init(); // 三个双PID参数初始化


    Uart3Init(115200); // 初始化Uart1
    PrintfInit(USART3); // printf 重定向到Uart

    // 發送歡迎訊息

    while (1) {
        // //hello_world();
        //printf("Hello\r\n");
        RunSystimer(); // 时间任务标志初始化  call 10ms
        //if (switchRX!=0) process_uart_command();  // 處理串口命令

        CLEAR_flag(); // 清除时间任务标志   clear flag
    }
}


