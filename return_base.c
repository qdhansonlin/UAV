#include "position_control.h"
#include "rc.h"
#include "height_ctrl.h"
#include "filter.h"
#include "return_base.h"
#include "stdlib.h"
#include "time.h"


					 
//int new_longitude, new_latitude;             //当前位置下坐标
u32 record_longitude[1000];
u32 record_latitude[1000];
int infi, max;

void position_record ()
{
	
	int temp_longitude, temp_latitude;
	
	temp_longitude =(int)gpsx.longitude;
	temp_latitude = (int)gpsx.latitude;
	
	
	if(temp_longitude!=0 && temp_latitude!=0){


		if(abs(temp_longitude - record_longitude[infi]) >=25 || abs(temp_latitude - record_latitude[infi])>=25){
		 if (infi <= 1){
			 infi=infi+1;
		   record_longitude[infi] = temp_longitude;
		   record_latitude[infi]  = temp_latitude;}
		 else if (abs(temp_longitude - record_longitude[infi]) <= 48 && abs(temp_latitude - record_latitude[infi]) <= 48 && infi <= 1000){
			 infi=infi+1;
		   record_longitude[infi] = temp_longitude;
		   record_latitude[infi]  = temp_latitude;
			 if(max < infi){ max = infi;}
		 }
	}
}
}

void position_return()
{
	if(infi >= max){
		infi = max - 1;}
	else if(abs(STOP_longitude - record_longitude[infi]) >=7 || abs(STOP_latitude - record_latitude[infi]) >= 7){
		if(abs(t_longitude - record_longitude[infi])<=48 && abs(t_latitude - record_latitude[infi]) <= 48 && abs(t_longitude - STOP_longitude)<= 5 && abs(t_latitude - STOP_latitude) <= 5 ){
  		if(infi >=1){
		STOP_longitude = record_longitude[infi];
		STOP_latitude  = record_latitude[infi];
			infi = infi - 1;
		}
			else{
				
	}
}
		}
	}
