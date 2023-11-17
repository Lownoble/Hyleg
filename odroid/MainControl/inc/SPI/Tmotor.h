#ifndef Tmotor_H
#define Tmotor_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define P_MIN  	-12.5
#define P_MAX  	12.5
#define V_MIN 	-23.24
#define V_MAX 	23.24
#define T_MIN 	-18
#define T_MAX   18
#define KP_MIN  0
#define KP_MAX 	500
#define KD_MIN 	0
#define KD_MAX 	5


typedef struct LTmotor
{
	int ID;
	float position;
	float velocity;
	float current;
}LTmotor;


typedef struct LMotorCmd
{
	int ID;
    int mode;
	float q;
	float dq;
	float tau;
	float Kp;
	float Kd;
}LMotorCmd;

typedef struct LMotorState
{
	int ID;
    int mode;
	float q;
	float dq;
	float ddq;
	float current;
	float none;
}LMotorState;

#endif