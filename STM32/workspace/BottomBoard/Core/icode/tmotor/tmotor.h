/*
 * tmotor.h
 */

#ifndef ICODE_TMOTOR_TMOTOR_H_
#define ICODE_TMOTOR_TMOTOR_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "usart.h"
#include "can.h"

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

#define INIT_ANGLE_1 45.0/180*M_PI
#define INIT_ANGLE_2 6.7/180*M_PI

#define ENABLE_MOTOR  0

typedef struct Tmotor
{
	int ID;
	float position;
	float velocity;
	float current;
	float kp;
	float kd;
	float t_ff;
}Tmotor;

typedef struct MotorCmd{
	unsigned int ID;
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

}MotorCmd;

typedef struct MotorState{
	unsigned int ID;
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float current;
    float none;
}MotorState;


extern Tmotor motor[5];
extern MotorCmd motorCmd[5];
extern MotorState motorState[5];
extern int enable_flag;

int can_send(int motor_address,unsigned char* send_buf,int send_len);
int motor_enable(int motor_address);
void motor_enable_all();
int motor_disable(int motor_address);
int motor_setzero(int motor_address);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff);
struct Tmotor unpack_RX(unsigned char rx_buf[6]);
void motor_setdata(unsigned char rx_buf[6]);
void motor_setCmd();
void motor_control();
void motor_init();
void state_passive();
#endif /* ICODE_TMOTOR_TMOTOR_H_ */
