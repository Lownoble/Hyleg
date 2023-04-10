/*
 * trajectory.c
 *
 *      Author: DZ
 */
#include "main.h"
#include "math.h"

int tra_cnt = 50;
int stand_flag = 1;
int swing_flag = 0;
float stand_trajectory[100][2];
float swing_trajectory[100][2];

//正运动学
float* FK(float theta1,float theta2)
{
	float hip_x = 60 * cos( theta2);
	float hip_y = -60 * sin(theta2);

	float knee_x = -300 * sin(theta1);
	float knee_y = -300 * cos(theta1);

	float foot_x = knee_x + 5*hip_x;
	float foot_y = knee_y + 5*hip_y;

	static float position[2];
	position[0] = foot_x; position[1] = foot_y;
	return position;
}

//逆运动学
float* IK(float x, float y)
{
	float theta1 = 2*atan(((600*x*(600*y + sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000))))/(pow(x,2) + 600*x + pow(y,2)) - 600*y - 600*x + (pow(x,2)*(600*y + sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000))))/(pow(x,2) + 600*x + pow(y,2)) + (pow(y,2)*(600*y + sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000))))/(pow(x,2) + 600*x + pow(y,2)))/(pow(x,2) + pow(y,2) - 600*y));
	float theta2 = -2*atan((600*y + sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000)))/(pow(x,2) + 600*x + pow(y,2)));
	static float theta[2];
	theta[0] = theta1; theta[1] = theta2;
	return theta;
}

/**
 * [轨迹规划]
 * @param   	 H     					 		[机器人高度]
 * @param   	 HF		   					 	[足端抬脚高度]
 * @param   	 LF     					 	[向前迈步距离]
 * @param   	 LB     					 	[向后迈步距离]
 * @return       void
 */
void trajectory(float H, float HF, float LF, float LB)
{
	int T = 100;
	for (int i = 0; i<T; i++){
	    float stand_x = (LB+LF)*((1.0*T-i)/T+0.5/PI*sin(2.0*PI*i/T))-LB;
	    float stand_y = -H;
	    float swing_x = (LB+LF)*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T))-LB;
	    float swing_y;
	    if(i<T/2){
	    	swing_y = -H + 2*HF*(1.0*i/T-0.25/PI*sin(4.0*PI*i/T));
	    }
	    else{
	    	swing_y = -H + 2*HF*(1.0-1.0*i/T+0.25/PI*sin(4.0*PI*i/T));
	    }

	    float* stand_theta = IK(stand_x,stand_y);
	    stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE1;
	    stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE2;
	    float* swing_theta = IK(swing_x,swing_y);
	    swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE1;
	    swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE2;
	    DmaPrintf("%f %f %f %f\n",stand_trajectory[i][0],stand_trajectory[i][1],swing_trajectory[i][0],swing_trajectory[i][1]);
	    HAL_Delay(10);
	}
}

