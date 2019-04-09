#include "height_ctrl.h"
#include "position_control.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "filter.h"
#include "ctrl.h"
#include "ms5611.h"
#include "rc.h"

_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;

float baro_speed;

float height_ctrl_out;
float wz_acc;

static float start_height = 0;//////////////////////////////

//水平方向的速度与加速度数据用于位置控制
float north_acc,west_acc;
/*向北方向的加速度 单位mm/s2，速度 单位mm/s，上一次速度 */
float north_acc_mms2,north_speed,north_speed_old;
/*向西方向的加速度 单位mm/s2，速度 单位mm/s，上一次速度 */
float west_acc_mms2,west_speed,west_speed_old;

#define BARO_SPEED_NUM 100
float baro_speed_arr[BARO_SPEED_NUM + 1];
u16 baro_cnt[2];

#define EXP_Z_SPEED  ( 4.0f *my_deathzoom( (thr-500), 50 ) )
void Height_Ctrl(float T,float thr)
{	
	static float wz_speed_t;
	static u8 height_ctrl_start_f;
	static u16 hc_start_delay;
		static u8 hs_ctrl_cnt;
	
	switch( height_ctrl_start_f )
	{
		case 0:
		if( mpu6050.Acc.z > 4000)
		{
			height_ctrl_start_f = 1;
		}	
		else if( ++hc_start_delay > 500 )
		{
			height_ctrl_start_f = 1;
		}
		
		break;
		
		case 1:
		
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (reference_v.z *mpu6050.Acc.z + reference_v.x *mpu6050.Acc.x + reference_v.y *mpu6050.Acc.y - 4096 ) - wz_acc );
		
		wz_speed_t += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(0.4f*(thr-500) - wz_speed_t);
		
		Moving_Average( (float)( baro_alt_speed *10),baro_speed_arr,BARO_SPEED_NUM, baro_cnt ,&baro_speed ); //单位mm/s
		if( height_ctrl_mode == 1)
		{
			if(baro_ctrl_start==1)//20ms
			{
				height_speed_ctrl(0.02f,thr,( EXP_Z_SPEED ),baro_speed);//baro_alt_speed *10);///
				baro_ctrl_start = 0;
				Baro_Ctrl(0.02f,thr);
			}		
		}
	
		else if( height_ctrl_mode == 2)
		{
			start_height = 0;////////////////////////////
			hs_ctrl_cnt++;
			hs_ctrl_cnt = hs_ctrl_cnt%10;
			if(hs_ctrl_cnt == 0)
			{
				height_speed_ctrl(0.02f,thr,0.4f*ultra_ctrl_out,ultra_speed);
			}
			
			if( ultra_start_f == 0 )
			{	
				
				Ultra_Ctrl(0.1f,thr);//超声波周期50ms
				ultra_start_f = -1;
			}
		}
	  ////////////////////////////////////////////////////////////	
		if(height_ctrl_mode)
		{		
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else
		{
			height_ctrl_out = thr;
			start_height = 0;
		}
		break;
		default: break;
	} 
}


void WZ_Speed_PID_Init()
{
	wz_speed_pid.kp = 0.3f;// *pid_setup.groups.hc_sp.kp; 
	wz_speed_pid.kd = 1.4f;//*pid_setup.groups.hc_sp.kd; 
	wz_speed_pid.ki = 0.12f;//*pid_setup.groups.hc_sp.ki; 
}

float wz_speed,wz_speed_old;

float wz_acc_mms2;


void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
	static float thr_lpf;
	float height_thr;
	static float hc_acc_i,wz_speed_0,wz_speed_1;
	//水平方向速度，用于位置控制
	static float north_acc_i,north_speed_0,north_speed_1;
  static float west_acc_i,west_speed_0,west_speed_1;
	
	height_thr = LIMIT( 2 * thr , 0, 600 );
	
	thr_lpf += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *( height_thr - thr_lpf );
	
	wz_acc_mms2 = (wz_acc/4096.0f) *10000 + hc_acc_i;//9800 *T;
	
	
	
	wz_speed_0 += my_deathzoom( (wz_acc_mms2 ) ,100) *T;
	
	hc_acc_i += 0.4f *T *( (wz_speed - wz_speed_old)/T - wz_acc_mms2 );
	hc_acc_i = LIMIT( hc_acc_i, -500, 500 );	
	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
	
	wz_speed_1 = wz_speed_0;
	
	if( ABS( wz_speed_1 ) < 50 )
	{
		wz_speed_1 = 0;
	}
	
	wz_speed_old = wz_speed;
	
	wz_speed = wz_speed_1;
	
	wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );
	wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid.kd * (-wz_acc_mms2) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
	
	//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
	wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid.kp *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);
	
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);

	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
	
	//加速度计与经纬度数据融合，计算水平速度
	/* 向北的加速度积分数据和纬度融合 */
	/* 没有重力数的据提取 */
	north_acc_mms2 = (north_acc/4096.0f) *10000 + north_acc_i;
	/* 死区范围外数据积分 */
	north_speed_0 += my_deathzoom( (north_acc_mms2 ) ,1000) * T;
	/* 加速度数据积分 */
	north_acc_i += 0.4f *T *( (north_speed - north_speed_old)/T - north_acc_mms2 );
	/* 加速度数据积分限幅 */
	north_acc_i = LIMIT( north_acc_i, -500, 500 );
	/* 高度数据积分滤波 */
	north_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( speed_latitude - north_speed_0  ) ;
	/* 计算出的速度赋值 */
	north_speed_1 = north_speed_0;
	/* 死区范围外数据有效 */
	if( ABS( north_speed_1 ) < 50 )north_speed_1 = 0;
	/* 记录上一次数据 */
	north_speed_old = north_speed;
	/* 计算出的速度赋值 */
	north_speed = north_speed_1;

	/* 向西的加速度积分数据和经度融合 */
	/* 没有重力数的据提取 */
	west_acc_mms2 = (west_acc/4096.0f) *10000 + west_acc_i;
	/* 死区范围外数据积分 */
	west_speed_0 += my_deathzoom( (west_acc_mms2 ) ,1000) * T;
	/* 加速度数据积分 */
	west_acc_i += 0.4f *T *( (west_speed - west_speed_old)/T - west_acc_mms2 );
	/* 加速度数据积分限幅 */
	west_acc_i = LIMIT( west_acc_i, -500, 500 );
	/* 高度数据积分滤波 */
	west_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( speed_longitude - west_speed_0  ) ;
	/* 计算出的速度赋值 */
	west_speed_1 = west_speed_0;
	/* 死区范围外数据有效 */
	if( ABS( west_speed_1 ) < 50 )west_speed_1 = 0;
	/* 记录上一次数据 */
	west_speed_old = west_speed;
	/* 计算出的速度赋值 */
	west_speed = west_speed_1;
}

u8 baro_ctrl_start;
float baro_height,baro_height_old;
float baro_measure;

void Baro_Ctrl(float T,float thr)
{
	

	/*        
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
  */
    

  
	
	if( (s16)start_height == 0 )
	{
		start_height = baroAlt;
	}		
	baro_measure = 10 *( baroAlt - start_height );
	
	baro_height += T *wz_speed;
	
	baro_height += 0.2f *3.14f *T *(baro_measure - baro_height);
				

        
}

#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_MAX_HEIGHT 1500   // mm
#define ULTRA_MIN_HEIGHT 20     //mm
#define ULTRA_INT        300    // 积分幅度

_st_height_pid_v ultra_ctrl;
_st_height_pid ultra_pid;

void Ultra_PID_Init()
{
	ultra_pid.kp = 1.5;
	ultra_pid.kd = 2.5;
	ultra_pid.ki = 0;
}

float exp_height_speed,exp_height;
float ultra_speed;
float ultra_dis_lpf;
float ultra_ctrl_out;

void Ultra_Ctrl(float T,float thr)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	
	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,50)/200.0f; //+-ULTRA_SPEEDmm / s
	exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
	
	if( exp_height > ULTRA_MAX_HEIGHT )
	{
		if( exp_height_speed > 0 )
		{
			exp_height_speed = 0;
		}
	}
	else if( exp_height < ULTRA_MIN_HEIGHT )
	{
		if( exp_height_speed < 0 )
		{
			exp_height_speed = 0;
		}
	}
	
	exp_height += exp_height_speed *T;
	
	/* 超声波定高最大高度限制 1500mm */
  if( exp_height > ULTRA_MAX_HEIGHT )
  {
    exp_height = ULTRA_MAX_HEIGHT;
  }
    /* 超声波定高最小高度限制 20mm */
  else if( exp_height < ULTRA_MIN_HEIGHT )
  {
    exp_height = ULTRA_MIN_HEIGHT;
  }
	
	if( thr < 100 )
	{
		exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);
	}
	
	ultra_sp_tmp = Moving_Median(0,5,ultra_delta/T); 

	
	
//	if( ultra_dis_tmp < 2000 )
//	{
		if( ABS(ultra_sp_tmp) < 100 )
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
		else
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
//	}
	
	ultra_dis_tmp = Moving_Median(1,5,ultra_distance);

	
//	if( ultra_dis_tmp < 2000 )
//	{
		
		if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
//	}
//	else
//	{
//		
//	}

	ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );
	
	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;
	
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);
	
	ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
	
	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
		
	ultra_ctrl_out = ultra_ctrl.pid_out;
	
	ultra_ctrl.err_old = ultra_ctrl.err;
}

