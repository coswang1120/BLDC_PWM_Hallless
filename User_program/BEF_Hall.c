//############################################################
// FILE: BEF_Hall.c
// Created on: 2017年1月18日
// Author: XQ
// summary: ThreeHall
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "BEF_Hall.h"
#include "Tim1_PWM.h"
#include "Task_function.h"

extern  Hall   BEF_ThreeCAP;
extern  logic   logicContr;

uint16_t   BEFHallK1=355; 		//Offset filter coefficient K1: 0.05/(T+0.05);
uint16_t   BEFHallK2=669 ;  	//Offset filter coefficient K2: T/(T+0.05);
 
void  BEF_ThreeCAPPara_init(void )
{
   BEF_ThreeCAP.Poles=4;
	 BEF_ThreeCAP.speed_coeff= 120000/BEF_ThreeCAP.Poles; // 93750测试 一次霍尔换向计算一次速度 
	  // 一次换向60度电角度内所花时间 pwm_count 是Hall_Three.Speed_count
	  // 90950是在开环电机1000RPM时候标定计算 用霍尔加示波器计算时间 一个周期示波器时间  
	  //一个周期示波器时间14.5ms假设60*(1/0.015)/p(极对数) = RPM =1000RPM 把90950此位置设置变量，Keil在线调试修改此数值 
    // Hall_Three.Speed_count*0.0000833(一次pwm80us) 一次霍尔换向所用时间
    // 一次霍尔是1/6周期  4对级数 1/24圈 一机械圈所花时间乘24   Hall_Three.Speed_count*0.00008333*24(一次pwm80us)  
    // 计算每秒多少圈1/(Hall_Three.Speed_count*0.00008333*24)
	  // 计算每分钟多少圈 1/(Hall_Three.Speed_count*0.0000833*24)*60 =60/(Hall_Three.Speed_count*0.0000833*24)
	  //=60/(Hall_Three.Speed_count*0.00008333*6(6次换向)*4(4对级数)) 
	  // 电机转速rpm=120000/4/Hall_Three.Speed_count
}


void  BEFHall_huanxkz(void)  // 一个PWM周期执行一次
{
	// GPIO读取反电动势过零比较处理换向，根据电机不同和过零电平的滤波电容设计，
	// 此过零信号不需要滞后30度，实际理论计算偏执角度大约在7度左右，此方案忽略处理
	// 在需要测量比较效率的控制器应用需要处理反电动势过零滞后角
	// 此开发板方案暂时不处理
	   if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==1) 
     BEF_ThreeCAP.HallUVW[0]=1;
		 else
		 BEF_ThreeCAP.HallUVW[0]=0; 
   	 if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)==1) 
     BEF_ThreeCAP.HallUVW[1]=1;
		 else
		 BEF_ThreeCAP.HallUVW[1]=0; 
		 if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)==1) 
     BEF_ThreeCAP.HallUVW[2]=1;
		  else
		 BEF_ThreeCAP.HallUVW[2]=0;  
	  BEF_ThreeCAP.Hall_State = BEF_ThreeCAP.HallUVW[0] +(BEF_ThreeCAP.HallUVW[1]<<1) +(BEF_ThreeCAP.HallUVW[2]<<2);
			
	  
			
			
			
   if(( logicContr.drive_car==1)&&(logicContr.Start_order==3 ))
 {
  if ( BEF_ThreeCAP.Hall_State!=BEF_ThreeCAP.OldHall_State )
    {     
	if(  logicContr.Run_mode==2 )    
 		 {
    	switch (BEF_ThreeCAP.Hall_State )
	      {
	     	 case  BEFCAP_numZ1 :   // 0x3
	    	      MOS_Q63PWM();
	    	      break;
	    	 case BEFCAP_numZ2:   //0x2
	    	      MOS_Q32PWM();
	    	  break;
	    	  case BEFCAP_numZ3:  // 0x6
	    	      MOS_Q25PWM();
	    	      break;
	    	  case BEFCAP_numZ4:  //0x4
	    	      MOS_Q54PWM();
	    	      break;
	    	  case BEFCAP_numZ5 :   //   0x5
	    	      MOS_Q41PWM();
	    	       break;
	    	  case BEFCAP_numZ6:   // 0x1
	    	       MOS_Q16PWM();
	    	     break;
               default:
	    	  	  {
	    	  	   //Stop_Motor();
	    	  	    BEF_ThreeCAP.Speed_RPM=0;
	    	  	   }
	    	       break;
	    	 }
      }
      else if ( logicContr.Run_mode==1 )
      {
    	switch (BEF_ThreeCAP.Hall_State )
    	  {
    		     	 case  BEFCAP_numF1 :   // 0x4
    		     		  MOS_Q54PWM();
    		    	      break;
    		    	 case BEFCAP_numF2:   //0x5
    		    		  MOS_Q41PWM();
    		    	  break;
    		    	  case BEFCAP_numF3:  // 0x1
    		    		  MOS_Q16PWM();
    		    	      break;
    		    	  case BEFCAP_numF4:  //0x3
    		    		  MOS_Q63PWM();
    		    	      break;
    		    	  case BEFCAP_numF5 :   //   0x2
    		    		  MOS_Q32PWM();
    		    	       break;
    		    	  case BEFCAP_numF6:   // 0x6
    		    		  MOS_Q25PWM();
    		    	     break;
    	          default:
    		    	  {
    		    	  	 //Stop_Motor();
    		    	  	 BEF_ThreeCAP.Speed_RPM=0;
    		    	  }
    		    	  break;
    		   }
         } 
    //计算转速，根据霍尔换相之间的时间 */		
    // 一阶数字低通把二个霍尔状态之间时间的计数平滑滤波，HallK1+ HallK2=1024，用y=a*y0+(1-a)*x;				 
							 
		  BEF_ThreeCAP.Speed_countFitter= _IQ10mpy(BEFHallK2, BEF_ThreeCAP.Speed_countFitter)+_IQ10mpy(BEFHallK1,  BEF_ThreeCAP.Speed_count);								  	 
      BEF_ThreeCAP.Speed_RPM = BEF_ThreeCAP.speed_coeff/BEF_ThreeCAP.Speed_countFitter;
		  BEF_ThreeCAP.Speed_RPMF= _IQ10mpy(BEFHallK2, BEF_ThreeCAP.Speed_RPMF)+_IQ10mpy(BEFHallK1,  BEF_ThreeCAP.Speed_RPM);	
  		BEF_ThreeCAP.Speed_count= 0;        
    }

     else  if ( BEF_ThreeCAP.Hall_State==BEF_ThreeCAP.OldHall_State )
     {
    	 BEF_ThreeCAP.Speed_count++;   
       if( BEF_ThreeCAP.Speed_count>=2000 )
    	 {
    		 BEF_ThreeCAP.Speed_count=0;
    		 BEF_ThreeCAP.Speed_RPMF= 0 ;
				 //Stop_Motor();
    		 BEF_ThreeCAP.step_angleFitter=0;	
				 BEF_ThreeCAP.Move_State=0;
				 logicContr.Start_order=0;

				 
         logicContr.drive_car=0;
				 logicContr.olddrive_car=logicContr.drive_car;
			




				 
      
       }
     }
	 }		 
   BEF_ThreeCAP.OldHall_State=BEF_ThreeCAP.Hall_State ;
}

 
//===========================================================================
// No more.
//===========================================================================
