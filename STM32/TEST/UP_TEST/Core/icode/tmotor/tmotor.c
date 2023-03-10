#include "tmotor.h"
#include "main.h"
#include "usart.h"
#include "can.h"

Tmotor motor1;
Tmotor motor2;
Tmotor motor3;
Tmotor motor4;

/**
 * [串口发送封装函数]
 * @param    ptr_can                 [串口设备文件描述符指针]
 * @param    send_buf                [需发送的数据缓冲区指针]
 * @param    send_len                [缓冲区是数据个数/字节数]
 * @return                        	 [发送结果标识，1为发送成功，0为发送失败]
 */

int can_send(int motor_address,unsigned char* send_buf,int send_len)
{
	int N = 0;
	printf("\nCAN_TX: ");
	for( int i = 0; i<send_len; i ++){
		printf("%x ",send_buf[i]);
	}
	N = CAN_TxMeg(motor_address,send_buf,send_len);
	while( N != 1 ){										//发送失败, 延时1个滴答, 再次发送
		HAL_Delay(1);
		N = CAN_TxMeg(motor_address,send_buf,send_len);
	}
	return 1;
}

/**
 * [电机使能通信]
 * @param    motor_address      	  [驱动器地址]
 * @return                         	  [命令发送结果,1 为发送成功,0 为发送失败]
 */
int motor_enable(int motor_address)
{
	if( motor_address <0 || motor_address >255)
	{
		return 0;
	}
	unsigned char send_buf[8]={0};
	send_buf[0] = 0xFF;
	send_buf[1] = 0xFF;
	send_buf[2] = 0xFF;
	send_buf[3] = 0xFF;
	send_buf[4] = 0xFF;
	send_buf[5] = 0xFF;
	send_buf[6] = 0xFF;
	send_buf[7] = 0xFC;
	if(can_send(motor_address,send_buf,8))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * [电机失能通信]
 * @param    motor_address     					  [驱动器地址]
 * @return                          			  [命令发送结果,1 为发送成功,0 为发送失败]
 */
int motor_disable(int motor_address)
{
	if(motor_address <0 || motor_address >255)
	{

		return 0;
	}
	unsigned char send_buf[8]={0};
	send_buf[0] = 0xFF;
	send_buf[1] = 0xFF;
	send_buf[2] = 0xFF;
	send_buf[3] = 0xFF;
	send_buf[4] = 0xFF;
	send_buf[5] = 0xFF;
	send_buf[6] = 0xFF;
	send_buf[7] = 0xFD;
	if(can_send(motor_address,send_buf,8))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
/// Converts a float to an unsigned int, given range and number of bits ///
{
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
/// converts unsigned int to float, given range and number of bits ///
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * [发送代码]
 * @param    motor_address      			    [驱动器地址]
 * @return                            			[命令发送结果,1 为发送成功,0 为发送失败]
 */
int pack_TX(int motor_address, float p_des, float v_des, float kp, float kd, float t_ff)
{
	unsigned char send_buf[8]={0};
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
		
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    send_buf[0] = p_int>>8; 					//位置高 8
    send_buf[1] = p_int&0xFF;					//位置低 8
    send_buf[2] = v_int>>4; 					//速度高 8 位
    send_buf[3] = ((v_int&0xF)<<4)|(kp_int>>8); //速度低 4 位 KP 高 4 位
    send_buf[4] = kp_int&0xFF; 					//KP 低 8 位
    send_buf[5] = kd_int>>4;					//Kd 高 8 位
    send_buf[6] = ((kd_int&0xF)<<4)|(t_int>>8); //KP 低 4 位扭矩高 4 位
    send_buf[7] = t_int&0xff; //扭矩低 8 
		
		if(can_send(motor_address,send_buf,8))
		{
			return 1;
		}
		else
		{
			return 0;
		}
}

/**
 * [接收代码]
 * @param    ptr_can                  [串口文件描述符的指针]
 * @param    motor_address       	  [驱动器地址]
 * @return                            [命令发送结果,1 为发送成功,0 为发送失败]
 */
struct Tmotor unpack_RX(unsigned char rx_buf[6])
/// unpack int from can buffer ///
{
	struct Tmotor reply;
    reply.ID = rx_buf[0]; 						//驱动 ID 号
    int p_int = (rx_buf[1]<<8)|rx_buf[2]; 		//电机位置数据
    int v_int = (rx_buf[3]<<4)|(rx_buf[4]>>4);  //电机速度数据
    int t_int = ((rx_buf[4]&0xF)<<8)|rx_buf[5]; //电机扭矩数据
	
    /// convert ints to floats ///
    reply.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    reply.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    reply.current = uint_to_float(t_int, T_MIN, T_MAX, 12);
		
	return reply;
}

void motor_setdata(int motor_address, unsigned char rx_buf[6])
{
	switch (motor_address)
	{
	case 1: motor1 = unpack_RX(rx_buf); break;
	case 2: motor2 = unpack_RX(rx_buf); break;
	case 3: motor3 = unpack_RX(rx_buf); break;
	case 4: motor4 = unpack_RX(rx_buf); break;
	}
}



