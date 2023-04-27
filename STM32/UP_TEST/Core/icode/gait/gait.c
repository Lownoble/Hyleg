/*
 * gait.c
 *
 */
#include "gait.h"
#include "main.h"

Leg Leg_right;
Leg Leg_left;

void gait_init(){
	for(int i=0;i<100;i++){
		if(Leg_left.stand_flag){
			motor_setdes(motor[1], Leg_left.stand_trajectory[Leg_left.count][0]);
			motor_setdes(motor[2], Leg_left.stand_trajectory[Leg_left.count][1]);
			Leg_left.count ++;
		}
		if(Leg_left.swing_flag){
			motor_setdes(motor[1], Leg_left.swing_trajectory[Leg_left.count][0]);
			motor_setdes(motor[2], Leg_left.swing_trajectory[Leg_left.count][1]);
			Leg_left.count ++;
		}
		if(Leg_right.stand_flag){
			motor_setdes(motor[3], Leg_right.stand_trajectory[Leg_right.count][0]);
			motor_setdes(motor[4], Leg_right.stand_trajectory[Leg_right.count][1]);
			Leg_right.count ++;
		}
		if(Leg_right.swing_flag){
			motor_setdes(motor[3], Leg_right.swing_trajectory[Leg_right.count][0]);
			motor_setdes(motor[4], Leg_right.swing_trajectory[Leg_right.count][1]);
			Leg_right.count ++;
		}
		motor_control();
		HAL_Delay(10);
	}
	Leg_left.count=0; Leg_left.stand_flag=1; Leg_left.swing_flag=0;
	Leg_right.count=0; Leg_right.stand_flag=0; Leg_right.swing_flag=1;
	Leg_left.H=500; Leg_left.HF=100; Leg_left.LF=100; Leg_left.LB=100;
	Leg_right.H=500; Leg_right.HF=100; Leg_right.LF=100; Leg_right.LB=100;
	trajectory_left(Leg_left.H,Leg_left.HF,Leg_left.LF,Leg_left.LB);
	trajectory_right(Leg_right.H,Leg_right.HF,Leg_right.LF,Leg_right.LB);
}


void speed_up(int dv){


}

void speed_down(int dv){

}

