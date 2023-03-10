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


typedef struct Tmotor
{
	int ID;
	float position;
	float velocity;
	float current;
}Tmotor;


int can_send(int motor_address,unsigned char* send_buf,int send_len);
int motor_enable(int motor_address);
int motor_disable(int motor_address);

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff);
struct Tmotor unpack_RX(unsigned char rx_buf[6]);
void motor_setdata(int motor_address, unsigned char rx_buf[6]);
#endif
