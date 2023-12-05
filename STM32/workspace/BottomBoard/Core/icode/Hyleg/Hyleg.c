/*
 * Hyleg.c
 *
 */

#include "Hyleg.h"

float* FK(float theta1,float theta2)
{
	float hip_x = 60 * cos(theta1+theta2);
	float hip_y = -60 * sin(theta1+theta2);

	float knee_x = -300 * sin(theta1);
	float knee_y = -300 * cos(theta1);

	float foot_x = knee_x + 5*hip_x;
	float foot_y = knee_y + 5*hip_y;

	static float position[2];
	position[0] = foot_x; position[1] = foot_y;
	return position;
}

