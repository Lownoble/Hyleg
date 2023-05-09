#include "tmotor.h"
#include "main.h"
#include "usart.h"
#include "can.h"

Tmotor motor[5];
int enable_flag = 0;

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
	enable_flag = 1;
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
	HAL_Delay(10);
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
	motor[1].ID = 1; motor[1].kp = 0; motor[1].kd = 0.5;
	motor[2].ID = 2; motor[2].kp = 0; motor[2].kd = 0.5;
	motor[3].ID = 3; motor[3].kp = 0; motor[3].kd = 0.5;
	motor[4].ID = 4; motor[4].kp = 0; motor[4].kd = 0.5;
	motor_enable(1);motor_enable(2);motor_enable(3);motor_enable(4);
	//	//设置运动PID
	for(int i = 0; i<20; i++)
	{
		motor[1].kp = motor[1].kp+i;
		motor[2].kp = motor[2].kp+i;
		motor[3].kp = motor[3].kp+i;
		motor[4].kp = motor[4].kp+i;
		DmaPrintf("%d\n",20-i);
		HAL_Delay(100);
		motor_control();
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
	CL_flag = 0;
	CR_flag = 0;
	enable_flag = 0;
	unsigned char send_buf[8]={0};
	send_buf[0] = 0xFF;
	send_buf[1] = 0xFF;
	send_buf[2] = 0xFF;
	send_buf[3] = 0xFF;
	send_buf[4] = 0xFF;
	send_buf[5] = 0xFF;
	send_buf[6] = 0xFF;
	send_buf[7] = 0xFD;
	DmaPrintf("Motor disable\n");
	HAL_Delay(10);
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
 * [电机设置零点通信]
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
	HAL_Delay(10);
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
 * [电机初始化通信]
 * @return                          			  [命令发送结果,1 为发送成功,0 为发送失败]
 */
int motor_init()
{
	motor_disable(1);motor_disable(2);motor_disable(3);motor_disable(4);
	HAL_Delay(100);
	//设置零点
	DmaPrintf("Waiting set zero...(Z/N)\n");
	while(USART1_RX_FLAG == 0){}
	if(USART1_RX_BUF[0]=='Z'){
		motor_setzero(1);motor_setzero(2);motor_setzero(3);motor_setzero(4);
		DmaPrintf("Set zero success!\n");
		HAL_Delay(100);
	}
	USART1_RX_CNT = 0;
	USART1_RX_FLAG = 0;
	USART1_RX_BUF[0] = 0;


	//运动轨迹生成
//	Leg_left.count=0; Leg_left.stand_flag=0; Leg_left.swing_flag=1;
//	Leg_right.count=0; Leg_right.stand_flag=1; Leg_right.swing_flag=0;
//	Leg_left.H=500; Leg_left.HF=100; Leg_left.LF=100; Leg_left.LB=0;
//	Leg_right.H=500; Leg_right.HF=100; Leg_right.LF=0; Leg_right.LB=100;

	//Leg_left.count = 50;
	Leg_left.count=0;
	Leg_left.stand_flag=1; Leg_left.swing_flag=0;
	//Leg_right.count = 50;
	Leg_right.count=(int)((BOTH_RATIO+1)*100/2);
	Leg_right.stand_flag=1; Leg_right.swing_flag=0;
	Leg_left.H=540; Leg_left.HF=80; Leg_left.LF=100; Leg_left.LB=100;
	Leg_right.H=540; Leg_right.HF=80; Leg_right.LF=100; Leg_right.LB=100;


	trajectory_walk();
	//trajectory_circle(450, 50);
	//trajectory_square(500, 100, 50);
	//trajectory_horizontal(540, 200);
	//trajectory_vertiacal(540, 80);

	//设置初始化PID
	motor[1].ID = 1; motor[1].kp = 0.2; motor[1].kd = 0.5;
	motor[2].ID = 2; motor[2].kp = 0.2; motor[2].kd = 0.5;
	motor[3].ID = 3; motor[3].kp = 0.2; motor[3].kd = 0.5;
	motor[4].ID = 4; motor[4].kp = 0.2; motor[4].kd = 0.5;
	HAL_Delay(100);

	//使能电机运动至初始位置
	DmaPrintf("Ready to start...(Y/N)\n");
	while(USART1_RX_BUF[0]!='Y'){}
	USART1_RX_CNT = 0;
	USART1_RX_FLAG = 0;
	USART1_RX_BUF[0] = 0;
	USART1_RX_BUF[1] = 0;
	HAL_Delay(100);

	motor_enable(1);motor_enable(2);motor_enable(3);motor_enable(4);
	if(Leg_left.stand_flag){
		motor[1].p_des = Leg_left.stand_trajectory[Leg_left.count][0];
		motor[2].p_des = Leg_left.stand_trajectory[Leg_left.count][1];
	}
	if(Leg_left.swing_flag){
		motor[1].p_des = Leg_left.swing_trajectory[Leg_left.count][0];
		motor[2].p_des = Leg_left.swing_trajectory[Leg_left.count][1];
	}
	if(Leg_right.stand_flag){
		motor[3].p_des = Leg_right.stand_trajectory[Leg_right.count][0];
		motor[4].p_des = Leg_right.stand_trajectory[Leg_right.count][1];
	}
	if(Leg_right.swing_flag){
		motor[3].p_des = Leg_right.swing_trajectory[Leg_right.count][0];
		motor[4].p_des = Leg_right.swing_trajectory[Leg_right.count][1];
	}
	motor_control();
	//	//设置运动PID
	for(int i = 0; i<20; i++)
	{
		motor[1].kp = motor[1].kp+i;
		motor[2].kp = motor[2].kp+i;
		motor[3].kp = motor[3].kp+i;
		motor[4].kp = motor[4].kp+i;
		DmaPrintf("%d\n",20-i);
		HAL_Delay(100);
		motor_control();
	}

	DmaPrintf("motor INIT DOWN\n");
	HAL_Delay(100);

//	DmaPrintf("Waiting to start...(Y/N)\n");
//	while(USART1_RX_BUF[0]!='Y'){}
//	USART1_RX_CNT = 0;
//	USART1_RX_FLAG = 0;
//	USART1_RX_BUF[0] = 0;
//	USART1_RX_BUF[1] = 0;
//	HAL_Delay(100);
//	DmaPrintf("Starting\n");
//	HAL_Delay(100);
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
 * @param    rx_buf                   [接收字符串]
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


/**
 * [设置数据]
 * @param    rx_buf                   [接收字符串]
 * @return                            [命令发送结果,1 为发送成功,0 为发送失败]
 */
void motor_setdata(unsigned char rx_buf[6])
{
	struct Tmotor reply;
	reply = unpack_RX(rx_buf);
	if(reply.ID == 8)	reply.ID = 4;
	motor[reply.ID].position = reply.position;
	motor[reply.ID].velocity = reply.velocity;
	motor[reply.ID].current = reply.current;

}


/**
 * [电机控制驱动]
 */
void motor_control()
{
	if(ENABLE_MOTOR){
	pack_TX(motor[1].ID, motor[1].p_des, motor[1].v_des, motor[1].kp, motor[1].kd, motor[1].t_ff);
	pack_TX(motor[2].ID, motor[2].p_des, motor[2].v_des, motor[2].kp, motor[2].kd, motor[2].t_ff);
	pack_TX(motor[3].ID, motor[3].p_des, motor[3].v_des, motor[3].kp, motor[3].kd, motor[3].t_ff);
	pack_TX(motor[4].ID, motor[4].p_des, motor[4].v_des, motor[4].kp, motor[4].kd, motor[4].t_ff);
	}
}


/**
 * [电机限位]
 * @param    ID				      	  [电机ID]
 * @param    p				      	  [电机位置值]
 * @param    p_des				      [电机期望位置值]
 * @return                         	  [命令发送结果,0 为正常，1 为超出位置限位,2 为超出速度限位]
 */
int motor_limit(int ID, float p, float p_des){
	//位置限位
	switch(ID){
	case 1:
		if(p < (MIN_ANGLE_L1-INIT_ANGLE_L1) ||p > (MAX_ANGLE_L1-INIT_ANGLE_L1)){
			DmaPrintf("1 position limit\n");
			HAL_Delay(10);
			return 1;
		}
		break;
	case 2:
		if(p < (MIN_ANGLE_L2-INIT_ANGLE_L2) ||p > (MAX_ANGLE_L2-INIT_ANGLE_L2)){
			DmaPrintf("2 position limit\n");
			HAL_Delay(10);
			return 1;
		}
		break;
	case 3:
		if(p < (MIN_ANGLE_R1-INIT_ANGLE_R1) ||p > (MAX_ANGLE_R1-INIT_ANGLE_R1)){
			DmaPrintf("3 position limit\n");
			HAL_Delay(10);
			return 1;
		}
		break;
	case 4:
		if(p < (MIN_ANGLE_R2-INIT_ANGLE_R2) ||p > (MAX_ANGLE_R2-INIT_ANGLE_R2)){
			DmaPrintf("4 position limit\n");
			HAL_Delay(10);
			return 1;
		}
		break;
	}
	//速度限位
	if( fabs(p_des-p) > V_LIMIT){
		DmaPrintf("velocity limit\n");
		HAL_Delay(10);
		return 2;
	}
	return 0;
}


/**
 * [电机设置期望位置]
 */
int motor_setdes(Tmotor M, float point)
{
	//位置限位
	if(motor_limit(M.ID, point, M.p_des) == 1) return 1;
	//速度限位
	if(motor_limit(M.ID, point, M.p_des) == 2){
		if( (M.p_des-point) > V_LIMIT){
			motor[M.ID].p_des -= V_LIMIT;
			return 2;
		}
		if( (point-M.p_des) > V_LIMIT){
			motor[M.ID].p_des += V_LIMIT;
			return 2;
		}
	}
	motor[M.ID].p_des = point;
	return 0;

}

