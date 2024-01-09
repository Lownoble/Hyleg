/*
 * Hyleg.c
 *
 */

#include "Hyleg.h"

float* FK(float theta1,float theta2)
{
	float L = 250;

	float foot_x = -L * sin(theta1) + L*cos(theta1+theta2);
	float foot_y = -L * cos(theta1) - L*sin(theta1+theta2);

	static float position[2];
	position[0] = foot_x; position[1] = foot_y;
	return position;
}

void calVelocity(float q1, float q2, float dq1, float dq2, float* velocity){
	float L = 250;
	velocity[0] = (-L*cos(q1)-L*sin(q1)*cos(q2)-L*cos(q1)*sin(q2))*dq1 + (-L*cos(q1)*sin(q2)-L*sin(q1)*cos(q2))*dq2;
	velocity[1] = ( L*sin(q1)-L*cos(q1)*cos(q2)+L*sin(q1)*sin(q2))*dq1 + ( L*sin(q1)*sin(q2)-L*cos(q1)*cos(q2))*dq2;
}
