/*******************************************************************************************
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * Author  : CSE 217
**********************************************************************************/
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "ms5611.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "position_control.h"

s16 loop_cnt;
s16 flag = 0;

loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_100ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}
float test[5];

void Duty_1ms()
{
//		if(flag == 0)
//	{ 
//	  LED5_ON;
//		flag = 1;
//	}
//	else
//	{
//		LED5_OFF;
//    flag = 0;
//	}
		
	Get_Cycle_T(1);
	LED_Display( LED_Brightness );								//20级led渐变显示
	//LIGHT_DT_Data_Exchange();												//数传通信定时调用
		
}


void Duty_2ms()
{

	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0); 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理

	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	
	
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{

	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2);								//获取外环准确的执行周期
	
	test[2] = GetSysTime_us()/1000000.0f;
	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

	//printf("Gyro  x:%f y:%f z:%f \r\n",mpu6050.Gyro_deg.x,mpu6050.Gyro_deg.y,mpu6050.Gyro_deg.z);
	
 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
}

void Duty_10ms()
{
 		if( MS5611_Update() ) 				//更新ms5611气压计数据
		{	
			baro_ctrl_start = 1;  //20ms
		}
		LIGHT_AK8975_Read();			//获取电子罗盘数据
	  
}

void Duty_20ms()
{
	//Parameter_Save();	

}

void Duty_50ms()
{
	Mode();
	LED_Duty();								//LED任务
	Ultra_Duty();
	test[3] = GetSysTime_us()/1000000.0f;
//		printf("time : %f : ",test[3]);
//		printf("Ultra : %d\r\n",ultra_distance);
}

void Duty_100ms()
{
    /* 调用2号计时通道，用于计算两侧调用的时间间隔 */
    float position_loop_time=Get_Cycle_T(2);
    
    /* GPS模块 */
    Call_GPS();
	
	  /* 位置控制 */
    PositionControl_Mode(position_loop_time);//next step
	
		//GPS数据输出
//		printf("\r\nGPS data output\r\n");
//	  printf("GPS constant state : %u\r\n",gpsx.gpssta);//GPS当前状态
//	  printf("GPS fixmode : %u\r\n",gpsx.fixmode);//定位类型
//		printf("svnum : %u\r\n",gpsx.svnum);//可见卫星数目
//	  printf("latitude : %c%u\r\n",gpsx.nshemi,gpsx.latitude);//纬度
//	  printf("longtitude : %c%u\r\n",gpsx.ewhemi,gpsx.longitude);//经度
//	  printf("altitude : %d\r\n",gpsx.altitude);//海拔高度
//	  printf("speed : %u\r\n",gpsx.speed);//速度
//		printf("GPS output done\r\n\r\n");
}



void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{


 	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		if( loop.cnt_100ms >= 100 )
		{
			loop.cnt_100ms = 0;
			Duty_100ms();					//周期100ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	
	

