/*
 * gait.h
 */

#ifndef ICODE_GAIT_GAIT_H_
#define ICODE_GAIT_GAIT_H_
#include "../icode/trajectory/trajectory.h"


typedef struct Leg
{
	int ID;
	int stand_flag;
	int swing_flag;
	int count;
	float H;
	float HF;
	float LF;
	float LB;
	float v;
	float stand_trajectory[200][2];
	float swing_trajectory[100][2];
}Leg;

extern int CL_flag;
extern int CR_flag;

extern Leg Leg_right;
extern Leg Leg_left;



#endif /* ICODE_GAIT_GAIT_H_ */
