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
	theta1 += INIT_ANGLE_L1;
	theta2 = theta2 + theta1;
	float hip_x = 60 * cos(theta2);
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
		//stand_x = (LB+LF)*((1.0*T*BOTH_RATIO-i)/(T*BOTH_RATIO)+0.5/PI*sin(2.0*PI*i/(T*BOTH_RATIO)))-LB;
	    stand_x = (LB+LF)/(1.0*T*BOTH_RATIO)*((T*BOTH_RATIO)-i)-LB;
	    stand_y = -H;
	    stand_theta = IK(stand_x,stand_y);
	    Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
	    Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
//	    DmaPrintf("%d %f %f\n",i,Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1]);
//	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(1, Leg_left.stand_trajectory[i-1][0], Leg_left.stand_trajectory[i][0]);
			motor_limit(2, Leg_left.stand_trajectory[i-1][1], Leg_left.stand_trajectory[i][1]);
	    }
	}
	for(int i = 0; i<T; i++){
		//swing_x = (LB+LF)*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T))-LB;
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
//	    DmaPrintf("%d %f %f\n",i,Leg_left.swing_trajectory[i][0],Leg_left.swing_trajectory[i][1]);
//	    HAL_Delay(10);
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
	    //stand_x = (LB+LF)*((1.0*T*BOTH_RATIO-i)/(T*BOTH_RATIO)+0.5/PI*sin(2.0*PI*i/(T*BOTH_RATIO)))-LB;
		stand_x = (LB+LF)/(1.0*T*BOTH_RATIO)*((T*BOTH_RATIO)-i)-LB;
		stand_y = -H;

	    stand_theta = IK(stand_x,stand_y);
	    Leg_right.stand_trajectory[i][0] = -(stand_theta[0]-INIT_ANGLE_R1);
	    Leg_right.stand_trajectory[i][1] = -(stand_theta[1]-INIT_ANGLE_R2) - Leg_right.stand_trajectory[i][0];
//	    DmaPrintf("%d %f %f\n",i,Leg_right.stand_trajectory[i][0],Leg_right.stand_trajectory[i][1]);
//	    HAL_Delay(10);
	    if(i!=0){
			motor_limit(3, Leg_right.stand_trajectory[i-1][0], Leg_right.stand_trajectory[i][0]);
			motor_limit(4, Leg_right.stand_trajectory[i-1][1], Leg_right.stand_trajectory[i][1]);
	    }
	}
	for(int i = 0; i<T; i++){
		//swing_x = (LB+LF)*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T))-LB;
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
//	    DmaPrintf("%d %f %f\n",i,Leg_right.swing_trajectory[i][0],Leg_right.swing_trajectory[i][1]);
//	    HAL_Delay(10);
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


void trajectory_horizontal(float H, float length){
	Leg_left.count = 50;Leg_left.stand_flag=1; Leg_left.swing_flag=0;
	Leg_right.count = 50;Leg_right.stand_flag=1; Leg_right.swing_flag=0;
	int T = 100;
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<T; i++){
		//stand_x = length*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T)) - length/2;
		stand_x = 0;
		stand_y = -H;

		//swing_x = length/2 - length*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T));
		swing_x = 0;
		swing_y = -H;

		stand_theta = IK(stand_x,stand_y);
		Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
		Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
		Leg_right.stand_trajectory[i][0] = -(stand_theta[0]-INIT_ANGLE_R1);
		Leg_right.stand_trajectory[i][1] = -(stand_theta[1]-INIT_ANGLE_R2) - Leg_right.stand_trajectory[i][0];

		swing_theta = IK(swing_x,swing_y);
		Leg_left.swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
		Leg_left.swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2 - Leg_left.swing_trajectory[i][0];
		Leg_right.swing_trajectory[i][0] = -(swing_theta[0]-INIT_ANGLE_R1);
		Leg_right.swing_trajectory[i][1] = -(swing_theta[1]-INIT_ANGLE_R2) - Leg_right.swing_trajectory[i][0];

		DmaPrintf("%d %f %f %f %f\n",i,Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1],Leg_left.swing_trajectory[i][0],Leg_left.swing_trajectory[i][1]);
		HAL_Delay(10);
		DmaPrintf("%d %f %f %f %f\n",i,Leg_right.stand_trajectory[i][0],Leg_right.stand_trajectory[i][1],Leg_right.swing_trajectory[i][0],Leg_right.swing_trajectory[i][1]);
		HAL_Delay(10);
	}
}

void trajectory_vertiacal(float H, float width){
	int T = 100;
	float stand_x;float stand_y;float swing_x;float swing_y;
	float* stand_theta; float* swing_theta;
	for (int i = 0; i<T; i++){
		stand_x = 0;
		stand_y = -H + width*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T));

		swing_x = 0;
		swing_y = -H + width-width*(1.0*i/T-0.5/PI*sin(2.0*PI*i/T));

		stand_theta = IK(stand_x,stand_y);
		Leg_left.stand_trajectory[i][0] = stand_theta[0]-INIT_ANGLE_L1;
		Leg_left.stand_trajectory[i][1] = stand_theta[1]-INIT_ANGLE_L2 - Leg_left.stand_trajectory[i][0];
		Leg_right.stand_trajectory[i][0] = -(stand_theta[0]-INIT_ANGLE_R1);
		Leg_right.stand_trajectory[i][1] = -(stand_theta[1]-INIT_ANGLE_R2) - Leg_right.stand_trajectory[i][0];

		swing_theta = IK(swing_x,swing_y);
		Leg_left.swing_trajectory[i][0] = swing_theta[0]-INIT_ANGLE_L1;
		Leg_left.swing_trajectory[i][1] = swing_theta[1]-INIT_ANGLE_L2 - Leg_left.swing_trajectory[i][0];
		Leg_right.swing_trajectory[i][0] = -(swing_theta[0]-INIT_ANGLE_R1);
		Leg_right.swing_trajectory[i][1] = -(swing_theta[1]-INIT_ANGLE_R2) - Leg_right.swing_trajectory[i][0];

		DmaPrintf("%d %f %f %f %f\n",i,Leg_left.stand_trajectory[i][0],Leg_left.stand_trajectory[i][1],Leg_left.swing_trajectory[i][0],Leg_left.swing_trajectory[i][1]);
		HAL_Delay(10);
	}
}


float motor1_offset[220] = {-0.171650,	-0.172050,	-0.171500,	-0.170500,	-0.167000,	-0.162500,	-0.160300,	-0.158100,	-0.157000,	-0.156300,	-0.155750,	-0.153800,	-0.151700,	-0.149450,	-0.145100,	-0.143200,	-0.135350,	-0.119200,	-0.100600,	-0.060550,	-0.012100,	0.020500,	0.048700,	0.044600,	0.027500,	-0.003250,	-0.022000,	-0.025500,	-0.017250,	-0.001000,	0.008500,	0.013250,	0.012250,	0.010500,	0.005500,	0.001250,	0.000000,	0.000750,	0.002250,	0.001500,	0.000875,	0.002000,	0.002250,	0.001375,	-0.000750,	-0.002500,	-0.003625,	-0.001750,	0.000500,	0.005250,	0.008000,	0.007250,	0.002125,	-0.004500,	-0.006750,	-0.006750,	-0.004250,	0.000000,	0.003250,	0.003750,	0.001750,	-0.002250,	-0.005000,	-0.005000,	-0.003250,	-0.000250,	0.000500,	0.000750,	-0.002000,	-0.001750,	-0.001625,	-0.001000,	-0.000250,	-0.000000,	-0.000250,	0.003500,	0.011875,	0.022250,	0.028000,	0.032500,	0.039250,	0.042750,	0.044750,	0.044250,	0.044250,	0.044000,	0.041500,	0.039250,	0.036875,	0.035000,	0.034250,	0.033875,	0.033250,	0.032500,	0.032000,	0.030750,	0.030750,	0.030375,	0.028750,	0.027500,	0.025500,	0.021750,	0.019250,	0.014000,	0.008000,	0.004000,	0.003000,	0.002000,	0.001500,	0.001625,	0.001000,	0.000750,	-0.002000,	-0.002250,	-0.003750,	-0.004125,	-0.002750,	-0.001500,	-0.000625,	0.001000,	0.003000,	0.004375,	0.008000,	0.009250,	0.016250,	0.024000,	0.030250,	0.036250,	0.041000,	0.043000,	0.046750,	0.050000,	0.051500,	0.054750,	0.056250,	0.055250,	0.050000,	0.042500,	0.037750,	0.030375,	0.025000,	0.024000,	0.021125,	0.018250,	0.015500,	0.008625,	-0.000250,	-0.008750,	-0.018250,	-0.026500,	-0.032250,	-0.037750,	-0.041500,	-0.043500,	-0.046500,	-0.049750,	-0.053750,	-0.059875,	-0.067000,	-0.071500,	-0.077000,	-0.079250,	-0.081000,	-0.082625,	-0.085750,	-0.086750,	-0.088875,	-0.091750,	-0.093000,	-0.094000,	-0.095000,	-0.094000,	-0.095500,	-0.097000,	-0.097000,	-0.099375,	-0.102000,	-0.104250,	-0.108750,	-0.112750,	-0.114000,	-0.116875,	-0.117750,	-0.118750,	-0.121375,	-0.122500,	-0.123250,	-0.125000,	-0.124250,	-0.125000,	-0.127375,	-0.130500,	-0.134000,	-0.143625,	-0.153500,	-0.159500,	-0.164250,	-0.158750,	-0.153250,	-0.144125,	-0.143000,	-0.144500,	-0.147750,	-0.150000,	-0.151500,	-0.152625,	-0.155000,	-0.157000,	-0.160625,	-0.163000,	-0.164750,	-0.167000,	-0.168000,	-0.168250,	-0.168000,	-0.167250,	-0.168250,	-0.170625,	-0.170000,	-0.171250};
float motor2_offset[220] = {-0.141450,	-0.141900,	-0.144200,	-0.144600,	-0.145650,	-0.145300,	-0.145000,	-0.143000,	-0.141000,	-0.140000,	-0.135700,	-0.131500,	-0.128500,	-0.122550,	-0.117600,	-0.108500,	-0.097750,	-0.085000,	-0.072300,	-0.039900,	0.000100,	0.012100,	0.007400,	0.006300,	0.005400,	-0.001375,	-0.009500,	-0.009000,	-0.003125,	-0.002500,	-0.001500,	0.005375,	0.005250,	0.000750,	-0.002875,	-0.001750,	0.002000,	0.003125,	0.000750,	-0.000250,	0.000625,	0.001750,	0.001500,	0.001125,	-0.000500,	-0.000500,	0.000750,	0.000250,	0.001250,	0.000750,	0.000500,	0.001000,	0.000625,	0.000500,	0.000000,	0.000000,	-0.000500,	-0.000250,	-0.000125,	0.000500,	0.000000,	-0.002375,	-0.002250,	-0.000750,	-0.001250,	-0.002250,	-0.002500,	-0.002125,	0.000000,	-0.000500,	-0.000750,	-0.001750,	-0.002500,	-0.001000,	-0.002500,	-0.006500,	-0.016500,	-0.028500,	-0.037500,	-0.048375,	-0.061000,	-0.067000,	-0.074500,	-0.076000,	-0.074000,	-0.070500,	-0.062000,	-0.053750,	-0.051000,	-0.054000,	-0.056250,	-0.062375,	-0.070750,	-0.073000,	-0.071875,	-0.072250,	-0.071500,	-0.070875,	-0.071000,	-0.070750,	-0.069625,	-0.069000,	-0.067000,	-0.068000,	-0.067000,	-0.066000,	-0.067875,	-0.068000,	-0.068000,	-0.067125,	-0.066500,	-0.066250,	-0.063625,	-0.063000,	-0.061500,	-0.059625,	-0.057250,	-0.056500,	-0.056000,	-0.056750,	-0.056500,	-0.056500,	-0.059000,	-0.062000,	-0.067000,	-0.071000,	-0.075000,	-0.077500,	-0.079500,	-0.080750,	-0.081750,	-0.082000,	-0.083000,	-0.086625,	-0.089750,	-0.089000,	-0.086375,	-0.084000,	-0.084000,	-0.089000,	-0.097000,	-0.101250,	-0.110000,	-0.115000,	-0.117000,	-0.117375,	-0.116000,	-0.115000,	-0.114500,	-0.116000,	-0.119000,	-0.124000,	-0.130500,	-0.136000,	-0.142750,	-0.149000,	-0.152000,	-0.154500,	-0.156000,	-0.156000,	-0.156000,	-0.157000,	-0.158000,	-0.158000,	-0.159000,	-0.160000,	-0.162000,	-0.165000,	-0.167000,	-0.171500,	-0.176000,	-0.177000,	-0.176625,	-0.177000,	-0.177000,	-0.177500,	-0.178000,	-0.178000,	-0.178375,	-0.179250,	-0.180750,	-0.181000,	-0.181000,	-0.182000,	-0.182500,	-0.183000,	-0.184000,	-0.186000,	-0.189000,	-0.187250,	-0.185500,	-0.180000,	-0.173250,	-0.158250,	-0.141000,	-0.130000,	-0.122750,	-0.116000,	-0.116750,	-0.117000,	-0.113750,	-0.113250,	-0.113625,	-0.112000,	-0.112000,	-0.112875,	-0.114000,	-0.116000,	-0.117125,	-0.120750,	-0.123500,	-0.124250,	-0.126250,	-0.129750,	-0.130625,	-0.131750,	-0.133250,	-0.135500,	-0.140250,	-0.141000};
float motor3_offset[220] = {-0.005125,	-0.005250,	-0.004300,	-0.003300,	-0.001550,	-0.002400,	-0.002000,	-0.003500,	-0.007900,	-0.008700,	-0.007300,	-0.009400,	-0.011000,	-0.014850,	-0.021300,	-0.026500,	-0.033850,	-0.041000,	-0.043300,	-0.045550,	-0.046200,	-0.046000,	-0.042250,	-0.038500,	-0.034400,	-0.028000,	-0.024250,	-0.021250,	-0.018625,	-0.013000,	-0.008500,	-0.004125,	-0.001500,	0.003500,	0.013625,	0.022000,	0.028000,	0.035875,	0.043000,	0.049500,	0.057125,	0.065250,	0.069000,	0.074500,	0.081000,	0.083500,	0.088750,	0.094000,	0.098500,	0.106250,	0.112750,	0.117250,	0.120500,	0.123000,	0.124500,	0.124250,	0.124000,	0.124500,	0.127250,	0.131500,	0.136000,	0.143000,	0.148000,	0.150000,	0.151500,	0.153000,	0.152500,	0.153375,	0.155500,	0.157500,	0.161500,	0.166250,	0.169500,	0.172250,	0.175000,	0.176500,	0.175625,	0.175500,	0.176000,	0.167500,	0.172750,	0.175500,	0.178500,	0.184000,	0.184000,	0.191500,	0.207000,	0.214000,	0.216000,	0.207250,	0.201000,	0.197125,	0.197000,	0.199000,	0.199500,	0.200000,	0.199000,	0.200375,	0.203500,	0.206750,	0.212875,	0.217000,	0.219000,	0.221000,	0.221000,	0.221000,	0.221125,	0.222000,	0.223000,	0.224125,	0.226250,	0.228000,	0.229500,	0.228000,	0.227250,	0.224750,	0.222500,	0.221500,	0.219750,	0.218250,	0.218000,	0.217000,	0.214750,	0.213000,	0.210000,	0.206000,	0.201000,	0.185000,	0.153750,	0.125000,	0.075875,	0.030000,	0.001000,	-0.023750,	-0.040000,	-0.039750,	-0.018625,	-0.003000,	0.009750,	0.023875,	0.024250,	0.017500,	0.001875,	-0.012750,	-0.013250,	-0.011500,	-0.010000,	-0.009750,	-0.004625,	0.001000,	0.002250,	0.001250,	-0.000250,	-0.001250,	-0.001125,	-0.003000,	-0.003250,	-0.002750,	-0.003750,	-0.001000,	0.000750,	0.001250,	0.000750,	-0.000375,	-0.000250,	-0.001750,	-0.002500,	-0.001000,	-0.001000,	-0.001250,	-0.000250,	0.000250,	0.001250,	0.002000,	0.003000,	0.001000,	0.001000,	0.001000,	0.000625,	0.001000,	0.001000,	-0.000500,	0.001000,	0.000000,	-0.001000,	-0.003000,	-0.006750,	-0.009000,	-0.013000,	-0.015750,	-0.023500,	-0.030250,	-0.032750,	-0.032250,	-0.031000,	-0.030000,	-0.027000,	-0.027000,	-0.027250,	-0.027375,	-0.026000,	-0.024000,	-0.023000,	-0.023500,	-0.022000,	-0.020875,	-0.020000,	-0.019000,	-0.018000,	-0.016000,	-0.014500,	-0.012625,	-0.010750,	-0.005500,	-0.003750,	-0.001750,	-0.002750,	0.000250,	-0.002000,	-0.005000};
float motor4_offset[220] = {0.068225,	0.067450,	0.067100,	0.066900,	0.068000,	0.067000,	0.064400,	0.066900,	0.067600,	0.066800,	0.068800,	0.071900,	0.071400,	0.071250,	0.074500,	0.076500,	0.080600,	0.081300,	0.082900,	0.087500,	0.094100,	0.095500,	0.096350,	0.094500,	0.093000,	0.092000,	0.094000,	0.100000,	0.108625,	0.115000,	0.117250,	0.118875,	0.117000,	0.117000,	0.118000,	0.118750,	0.118250,	0.123750,	0.129750,	0.133500,	0.141500,	0.150000,	0.155500,	0.161500,	0.166000,	0.168000,	0.171250,	0.174500,	0.175750,	0.179750,	0.183500,	0.186500,	0.189000,	0.190000,	0.190500,	0.191500,	0.191000,	0.191000,	0.191000,	0.191000,	0.191000,	0.191500,	0.192000,	0.192000,	0.192000,	0.192000,	0.192000,	0.192125,	0.193000,	0.194000,	0.194375,	0.195000,	0.195000,	0.197000,	0.197750,	0.198000,	0.198000,	0.197750,	0.198000,	0.196375,	0.189000,	0.183250,	0.173500,	0.163000,	0.162750,	0.143000,	0.118000,	0.105000,	0.098000,	0.102000,	0.105500,	0.110625,	0.114500,	0.114250,	0.115500,	0.115000,	0.114500,	0.116500,	0.121000,	0.123000,	0.123500,	0.125750,	0.130000,	0.130000,	0.130000,	0.130000,	0.131375,	0.134750,	0.139250,	0.141375,	0.142000,	0.144500,	0.144750,	0.145750,	0.148250,	0.147000,	0.147000,	0.147000,	0.146875,	0.147500,	0.144250,	0.139000,	0.135000,	0.130000,	0.124375,	0.119000,	0.119000,	0.111500,	0.104500,	0.092750,	0.065000,	0.026000,	-0.002000,	-0.015000,	-0.008000,	-0.001750,	-0.001000,	0.000000,	0.002000,	0.009000,	0.007000,	0.004000,	0.002000,	-0.001000,	-0.002000,	-0.002750,	0.001750,	0.003000,	0.001500,	0.000000,	0.000750,	0.001375,	0.000000,	-0.001000,	-0.000625,	0.000000,	0.000750,	0.000375,	0.001000,	0.000500,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	0.000000,	-0.000500,	0.000000,	0.000000,	0.000500,	0.000000,	-0.000250,	-0.000125,	0.002000,	0.002000,	0.000500,	0.002250,	0.001250,	0.001250,	0.001000,	0.002000,	0.000125,	-0.002000,	-0.002000,	0.000000,	0.002000,	0.009000,	0.021375,	0.033250,	0.047000,	0.059000,	0.068500,	0.073750,	0.071000,	0.065000,	0.063000,	0.059750,	0.059000,	0.062000,	0.066750,	0.066000,	0.066500,	0.074625,	0.081250,	0.081000,	0.079500,	0.078000,	0.078000,	0.079000,	0.078000,	0.078000,	0.078500,	0.074750,	0.073750,	0.074500,	0.075750,	0.072000,	0.071250,	0.071250,	0.069000};

void trajectory_offset(){
	int T = 100;
	for (int i = 0; i<(int)(T*BOTH_RATIO); i++){
		Leg_left.stand_trajectory[i][0] -= motor1_offset[i+T];
		Leg_left.stand_trajectory[i][1] -= motor2_offset[i+T];
		Leg_right.stand_trajectory[i][0] -= motor3_offset[i+T];
		Leg_right.stand_trajectory[i][1] -= motor4_offset[i+T];

	}
	for (int i = 0; i<T; i++){
		Leg_left.swing_trajectory[i][0] -= motor1_offset[i];
		Leg_left.swing_trajectory[i][1] -= motor2_offset[i];
		Leg_right.swing_trajectory[i][0] -= motor3_offset[i];
		Leg_right.swing_trajectory[i][1] -= motor4_offset[i];
	}
}
void trajectory_walk(){
	trajectory_left(Leg_left.H,Leg_left.HF,Leg_left.LF,Leg_left.LB);
	trajectory_right(Leg_right.H,Leg_right.HF,Leg_right.LF,Leg_right.LB);
	//trajectory_offset();
}

void change_H(){
	float H;
	float KP_H = 0.5;
	if(Leg_left.stand_flag&&!Leg_right.stand_flag){
		float* position;
		position = FK(motor[1].position,motor[2].position);
		H = Leg_left.H - KP_H * (-position[1]-Leg_left.H);
		trajectory_left(H,Leg_left.HF,Leg_left.LF,Leg_left.LB);
	}
	if(Leg_right.stand_flag&&!Leg_left.stand_flag){
		float* position;
		position = FK(-motor[3].position,-motor[4].position);
		H = Leg_right.H - KP_H * (-position[1]-Leg_right.H);
		trajectory_right(H,Leg_right.HF,Leg_right.LF,Leg_right.LB);
	}
	if(Leg_left.stand_flag&&Leg_right.stand_flag){
		float* position1;
		float* position2;
		position1 = FK(motor[1].position,motor[2].position);
		position2 = FK(-motor[3].position,-motor[4].position);
		H = Leg_right.H - KP_H * (-(position1[1]+position2[1])/2-Leg_right.H);
		trajectory_left(H,Leg_left.HF,Leg_left.LF,Leg_left.LB);
		trajectory_right(H,Leg_right.HF,Leg_right.LF,Leg_right.LB);
	}
}
