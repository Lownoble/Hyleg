/*
 * trajectory.c
 *
 *      Author: DZ
 */
#include "main.h"
#include "math.h"

//int tra_cnt = 50;
//int stand_flag = 1;
//int swing_flag = 0;
float stand_trajectory[100][2];
float swing_trajectory[100][2];

int test_cnt = 0;
float test_trajectory[10]= {0.5, 0.51, 0.52, 0.53, 0.54, 0.55, 0.54, 0.53, 0.52, 0.51};

//正运动学
float* FK(float theta1,float theta2)
{
	float hip_x = -60 * sin(theta1-theta2);
	float hip_y = -60 * cos(theta1-theta2);

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
	//float theta1 = 2*atan(((pow(x,2)*sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000)))/(pow(x,2) + pow(y,2)) - 600*x + (pow(y,2)*sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000)))/(pow(x,2) + pow(y,2)))/(pow(x,2) + pow(y,2) - 600*y));

	//float theta2 = 2*atan(sqrt(-(pow(x,2) + pow(y,2))*(pow(x,2) + pow(y,2) - 360000))/(pow(x,2) + pow(y,2)));

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
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<T; i++){
		//摆线
	    stand_x = (LB+LF)*((1.0*T-i)/T+0.5/PI*sin(2.0*PI*i/T))-LB;
	    stand_y = -H;
	    swing_x = (LB+LF)*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T))-LB;
	    if(i<T/2){
	    	swing_y = -H + 2*HF*(1.0*i/T-0.25/PI*sin(4.0*PI*i/T));
	    }
	    else{
	    	swing_y = -H + 2*HF*(1.0-1.0*i/T+0.25/PI*sin(4.0*PI*i/T));
	    }

	    stand_theta = IK(stand_x,stand_y);
	    stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
	    stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2;
	    swing_theta = IK(swing_x,swing_y);
	    swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
	    swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2;

	    DmaPrintf("%d %f %f %f %f\n",i,stand_trajectory[i][0],stand_trajectory[i][1],swing_trajectory[i][0],swing_trajectory[i][1]);
	    HAL_Delay(10);
	}
}

void trajectory_left(float H, float HF, float LF, float LB)
{
	int T = 100;
	float v = (LB+LF)/(T*BOTH_RATIO);
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;

	for (int i = 0; i<(int)(T*BOTH_RATIO); i++){
		//摆线
	    stand_x = (LB+LF)/(1.0*T*BOTH_RATIO)*((T*BOTH_RATIO)-i)-LB;
	    stand_y = -H;
	    stand_theta = IK(stand_x,stand_y);
	    Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
	    Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
	    DmaPrintf("%d %f %f\n",i,Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1]);
	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(1, Leg_left.stand_trajectory[i-1][0], Leg_left.stand_trajectory[i][0]);
			motor_limit(2, Leg_left.stand_trajectory[i-1][1], Leg_left.stand_trajectory[i][1]);
	    }

	}
	for(int i = 0; i<T; i++){
	    swing_x = -6*(v*T+LB+LF)/pow(T,5)*pow((T-i),5)+15*(v*T+LB+LF)/pow(T,4)*pow((T-i),4)-10*(v*T+LB+LF)/pow(T,3)*pow((T-i),3)+v*(T-i)+LF;
	    if(i<T/2){
	    	swing_y = -H + 2*HF*(1.0*i/T-0.25/PI*sin(4.0*PI*i/T));
	    }
	    else{
	    	swing_y = -H + 2*HF*(1.0-1.0*i/T+0.25/PI*sin(4.0*PI*i/T));
	    }
	    swing_theta = IK(swing_x,swing_y);
	    Leg_left.swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
	    Leg_left.swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2 - Leg_left.swing_trajectory[i][0];
	    DmaPrintf("%d %f %f\n",i,Leg_left.swing_trajectory[i][0],Leg_left.swing_trajectory[i][1]);
	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(1, Leg_left.swing_trajectory[i-1][0], Leg_left.swing_trajectory[i][0]);
			motor_limit(2, Leg_left.swing_trajectory[i-1][1], Leg_left.swing_trajectory[i][1]);
	    }
	}

}

void trajectory_right(float H, float HF, float LF, float LB)
{
	int T = 100;
	float v = (LB+LF)/(T*BOTH_RATIO);
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<(int)(T*BOTH_RATIO); i++){
		//摆线
	    stand_x = (LB+LF)/(1.0*T*BOTH_RATIO)*((T*BOTH_RATIO)-i)-LB;
	    stand_y = -H;
	    stand_theta = IK(stand_x,stand_y);
	    Leg_right.stand_trajectory[i][0] = -(stand_theta[0]-INIT_ANGLE_R1);
	    Leg_right.stand_trajectory[i][1] = -(stand_theta[1]-INIT_ANGLE_R2) - Leg_right.stand_trajectory[i][0];
	    DmaPrintf("%d %f %f\n",i,Leg_right.stand_trajectory[i][0],Leg_right.stand_trajectory[i][1]);
	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(3, Leg_right.stand_trajectory[i-1][0], Leg_right.stand_trajectory[i][0]);
			motor_limit(4, Leg_right.stand_trajectory[i-1][1], Leg_right.stand_trajectory[i][1]);
	    }
	}
	for(int i = 0; i<T; i++){
	    swing_x = -6*(v*T+LB+LF)/pow(T,5)*pow((T-i),5)+15*(v*T+LB+LF)/pow(T,4)*pow((T-i),4)-10*(v*T+LB+LF)/pow(T,3)*pow((T-i),3)+v*(T-i)+LF;
	    if(i<T/2){
	    	swing_y = -H + 2*HF*(1.0*i/T-0.25/PI*sin(4.0*PI*i/T));
	    }
	    else{
	    	swing_y = -H + 2*HF*(1.0-1.0*i/T+0.25/PI*sin(4.0*PI*i/T));
	    }
	    swing_theta = IK(swing_x,swing_y);
	    Leg_right.swing_trajectory[i][0] = -(swing_theta[0]-INIT_ANGLE_R1);
	    Leg_right.swing_trajectory[i][1] = -(swing_theta[1]-INIT_ANGLE_R2) - Leg_right.swing_trajectory[i][0];
	    DmaPrintf("%d %f %f\n",i,Leg_right.swing_trajectory[i][0],Leg_right.swing_trajectory[i][1]);
	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(3, Leg_right.swing_trajectory[i-1][0], Leg_right.swing_trajectory[i][0]);
			motor_limit(4, Leg_right.swing_trajectory[i-1][1], Leg_right.swing_trajectory[i][1]);
	    }
	}
}


void trajectory_circle(float H, float R)
{
	int T = 100;
	float stand_x;float stand_y;float swing_x;float swing_y;
	float theta;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<T; i++){
		    //圆
		    theta = 1.0*PI*i/T;
		    stand_x = R - R*cos(theta);
		    stand_y = -H + R*sin(theta);
		    swing_x = R + R*cos(theta);
		    swing_y = -H - R*sin(theta);

		    stand_theta = IK(stand_x,stand_y);
		    Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
		    Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
		    swing_theta = IK(swing_x,swing_y);
		    Leg_left.swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
		    Leg_left.swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2 - Leg_left.swing_trajectory[i][0];
		    DmaPrintf("%f %f %f %f\n",Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1],swing_trajectory[i][0],swing_trajectory[i][1]);
		    HAL_Delay(10);
	}
}

void trajectory_square(float H, float square_length, float square_width)
{
	int T = 100;
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<T; i++){
		    //方
		    if(i<T/2){
		    	stand_x = 0;
		    	stand_y = -H + square_width*(2.0*i/T-0.5/PI*sin(4.0*PI*i/T));

		    	swing_x = square_length;
		    	swing_y = -H + square_width *((T-2.0*i)/T+0.5/PI*sin(4.0*PI*i/T));
		    }
		    else{

		    	stand_x = square_length*((2.0*i-T)/T-0.5/PI*sin(2.0*PI*(2.0*i-T)/T));
		    	stand_y = -H + square_width;

		    	swing_x = square_length - square_length*((2.0*i-T)/T-0.5/PI*sin(2.0*PI*(2.0*i-T)/T));
		    	swing_y = -H;
		    }

		    stand_theta = IK(stand_x,stand_y);
		    Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
		    Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
		    swing_theta = IK(swing_x,swing_y);
		    Leg_left.swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
		    Leg_left.swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2 - Leg_left.swing_trajectory[i][0];
		    DmaPrintf("%d %f %f %f %f\n",i,Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1],Leg_left.swing_trajectory[i][0],Leg_left.swing_trajectory[i][1]);
		    HAL_Delay(10);
	}
}
