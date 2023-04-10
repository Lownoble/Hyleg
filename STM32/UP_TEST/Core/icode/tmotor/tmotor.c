#include "tmotor.h"
#include "main.h"
#include "usart.h"
#include "can.h"

Tmotor motor1;
Tmotor motor2;
Tmotor motor3;
Tmotor motor4;
Tmotor motor5;

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
//	printf("\nCAN_TX: ");
//	for( int i = 0; i<send_len; i ++){
//		printf("%x ",send_buf[i]);
//	}
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
	//DmaPrintf("Motor enable\n");
	if(can_send(motor_address,send_buf,8))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void motor_enable_all()
{
	motor1.ID = 1; motor1.kp = 0; motor1.kd = 0.5;
	motor2.ID = 2; motor2.kp = 0; motor2.kd = 0.5;
	motor3.ID = 3; motor3.kp = 0; motor3.kd = 0.5;
	motor4.ID = 4; motor4.kp = 0; motor4.kd = 0.5;
	pack_TX(motor2.ID, motor2.p_des, motor2.v_des, motor2.kp, motor2.kd, motor2.t_ff);
	motor_enable(1);motor_enable(2);motor_enable(3);motor_enable(4);
	//	//设置运动PID
	for(int i = 0; i<20; i++)
	{
		motor1.kp = motor1.kp+i;
		motor2.kp = motor2.kp+i;
		motor3.kp = motor3.kp+i;
		motor4.kp = motor4.kp+i;
		DmaPrintf("%d\n",i);
		HAL_Delay(100);
		pack_TX(motor2.ID, motor2.p_des, motor2.v_des, motor2.kp, motor2.kd, motor2.t_ff);
		//motor_control();
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
	//DmaPrintf("Motor disable\n");
	pack_TX(motor_address, 0, 0, 1, 1, 0);
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
int motor_setzero(int motor_address)
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
	send_buf[7] = 0xFE;
	DmaPrintf("Motor set zero\n");
	if(can_send(motor_address,send_buf,8))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



int motor_init()
{
	motor_disable(1);motor_disable(2);motor_disable(3);motor_disable(4);
	HAL_Delay(100);
	//设置零点
	DmaPrintf("Waiting set zero...(Y/N)\n");
	while(USART1_RX_FLAG == 0){}
	if(USART1_RX_BUF[0]=='Y'){
		motor_setzero(1);motor_setzero(2);motor_setzero(3);motor_setzero(4);
		DmaPrintf("Set zero success!\n");
	}
	USART1_RX_CNT = 0;
	USART1_RX_FLAG = 0;
	USART1_RX_BUF[0] = 0;


	//运动轨迹生成
	trajectory(500,100, 100, 100);

	//设置初始化PID
	motor1.ID = 1; motor1.enable_flag = 0; motor1.kp = 0.2; motor1.kd = 0.5;
	motor2.ID = 2; motor2.enable_flag = 0; motor2.kp = 0.2; motor2.kd = 0.5;
	motor3.ID = 3; motor3.enable_flag = 0; motor3.kp = 0.2; motor3.kd = 0.5;
	motor4.ID = 4; motor4.enable_flag = 0; motor4.kp = 0.2; motor4.kd = 0.5;
	HAL_Delay(100);

	//使能电机运动至初始位置
	DmaPrintf("Ready to start...(Y/N)\n");
	while(USART1_RX_BUF[0]!='Y'){}
	USART1_RX_CNT = 0;
	USART1_RX_FLAG = 0;
	USART1_RX_BUF[0] = 0;
	HAL_Delay(100);

	motor_enable(1);motor_enable(2);motor_enable(3);motor_enable(4);
	motor1.p_des = stand_trajectory[tra_cnt][0];
	motor2.p_des = stand_trajectory[tra_cnt][1];
	motor3.p_des = swing_trajectory[tra_cnt][0];
	motor4.p_des = swing_trajectory[tra_cnt][1];

	motor_control();
	//	//设置运动PID
	for(int i = 0; i<20; i++)
	{
		motor1.kp = motor1.kp+i;
		motor2.kp = motor2.kp+i;
		motor3.kp = motor3.kp+i;
		motor4.kp = motor4.kp+i;
		DmaPrintf("%d\n",i);
		HAL_Delay(100);
		motor_control();
	}

	DmaPrintf("INIT DOWN");
	HAL_Delay(100);


	return 1;
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

void motor_setdata(unsigned char rx_buf[6])
{
	struct Tmotor reply;
	reply = unpack_RX(rx_buf);
//	if(reply.ID == 2)
//	{
//		motor2 = reply;
//	}
	switch (reply.ID)
	{
	case 1:
		motor1.position = reply.position;
		motor1.velocity = reply.velocity;
		motor1.current = reply.current;
		break;
	case 2:
		motor2.position = reply.position;
		motor2.velocity = reply.velocity;
		motor2.current = reply.current;
		break;
	case 3:
		motor3.position = reply.position;
		motor3.velocity = reply.velocity;
		motor3.current = reply.current;
		break;
	case 4:
		motor4.position = reply.position;
		motor4.velocity = reply.velocity;
		motor4.current = reply.current;
		break;
	}
}

void motor_control(){
	pack_TX(motor1.ID, motor1.p_des, motor1.v_des, motor1.kp, motor1.kd, motor1.t_ff);
	pack_TX(motor2.ID, motor2.p_des, motor2.v_des, motor2.kp, motor2.kd, motor2.t_ff);
	pack_TX(motor3.ID, motor3.p_des, motor3.v_des, motor3.kp, motor3.kd, motor3.t_ff);
	pack_TX(motor4.ID, motor4.p_des, motor4.v_des, motor4.kp, motor4.kd, motor4.t_ff);
}

