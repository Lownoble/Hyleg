#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "sys_time.h"
#include "spi_node.h"
#include "comm.h"
#include <time.h> 
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <pthread.h>


typedef struct{
	char connect;
	int loss_cnt;
	char mode;
	char up_mode;
	float rc_spd_b[3],rc_rate_b[3];
	
	char cmd_robot_state;
	float rc_spd_w[2],rc_att_w[2],rate_yaw_w;
	int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back;
	int key_ud_reg,key_lr_reg,key_x_reg,key_y_reg,key_a_reg,key_b_reg,key_ll_reg,key_rr_reg,key_st_reg,key_back_reg;
}_OCU;
_OCU ocu;

// ps命令查找与进程相关的PID号： 
// kill －9 324 
#define SERV_PORT   8889
//----------------------------UDP 通讯------------

float floatFromData( char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return *(float *)&i;
}

char charFromData( char *data,int* anal_cnt)
{
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	return *(data+temp);
}

int intFromData( char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return i;
}

void Anal_UDP(char *data_buf,char num)
{  
	char id;
	char temp_char=0,need_save=0;
	char sum = 0;
	char i,j;
	int anal_cnt=4;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1))){		
		i=0;
		return;		//判断sum
	}
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	{
		i=0;
		return;		//判断帧头
	}
  if(*(data_buf+2)==0x32)//ocu speed remote
  { 
		ocu.connect=1;
		ocu.loss_cnt=0;
		if(ocu.mode!=3){
			ocu.mode=2;		
			ocu.up_mode=charFromData(data_buf,&anal_cnt);
			temp_char=charFromData(data_buf,&anal_cnt);
			ocu.rc_spd_w[Xr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_spd_w[Yr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_att_w[PITr]=floatFromData(data_buf,&anal_cnt);
			ocu.rc_att_w[ROLr]=floatFromData(data_buf,&anal_cnt);
			ocu.rate_yaw_w=floatFromData(data_buf,&anal_cnt);
			ocu.key_st=charFromData(data_buf,&anal_cnt);
			ocu.key_back=charFromData(data_buf,&anal_cnt);
			ocu.key_lr=intFromData(data_buf,&anal_cnt);
			ocu.key_ud=intFromData(data_buf,&anal_cnt);
			ocu.key_x=charFromData(data_buf,&anal_cnt);
			ocu.key_y=charFromData(data_buf,&anal_cnt);
			ocu.key_b=charFromData(data_buf,&anal_cnt);
			ocu.key_a=charFromData(data_buf,&anal_cnt);
			ocu.key_ll=charFromData(data_buf,&anal_cnt);
			ocu.key_rr=charFromData(data_buf,&anal_cnt);
			printf("%d %d %d\n",ocu.key_ud,ocu.key_lr,ocu.key_x);
		}
	}
}

char RxBuffer_USB[255];
char RxStateUSB=0;
int _data_lenUSB=0;
int _data_cntUSB=0;
void UDP_RX_PROCESS(char* Buf, int Len)
{
	int i=0;
	char com_data;
	for(i=0;i<Len;i++)
	{  
		com_data=Buf[i]; 
		if(RxStateUSB==0&&com_data==0xAA)
		{
			RxStateUSB=1;
			RxBuffer_USB[0]=com_data;
		}
		else if(RxStateUSB==1&&com_data==0xAF)
		{
			RxStateUSB=2;
			RxBuffer_USB[1]=com_data;
		}
		else if(RxStateUSB==2&&com_data>0&&com_data<0XF1)
		{
			RxStateUSB=3;
			RxBuffer_USB[2]=com_data;
		}
		else if(RxStateUSB==3&&com_data<255)
		{
			RxStateUSB= 4;
			RxBuffer_USB[3]=com_data;
			_data_lenUSB = com_data;
			_data_cntUSB = 0;
		}
		else if(RxStateUSB==4&&_data_lenUSB>0)
		{
			_data_lenUSB--;
			RxBuffer_USB[4+_data_cntUSB++]=com_data;
			if(_data_lenUSB==0)
				RxStateUSB= 5;
		}
		else if(RxStateUSB==5)
		{
			RxStateUSB = 0;
			RxBuffer_USB[4+_data_cntUSB]=com_data;
      		Anal_UDP(RxBuffer_USB,_data_cntUSB+5);
			for(i=0;i<255;i++)
				RxBuffer_USB[i]=0;
		}
		else
			RxStateUSB = 0;
	}
}

int usb_send_cnt=0;
char SendBuff_USB[250];
void setDataInt(int i)
{
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

void setDataFloat(float f)
{
	int i = *(int *)&f;
	#if 0
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	#else
	SendBuff_USB[usb_send_cnt++] = ((i << 24) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 16) >> 24);
	SendBuff_USB[usb_send_cnt++] = ((i << 8) >> 24);
	SendBuff_USB[usb_send_cnt++] = (i >> 24);
	#endif
}

void data_per_usb_robot_state1(void)
{ 
	int i;	u8 sum = 0;
	u16 _cnt=0;
	int cnt_reg=0;

	cnt_reg=usb_send_cnt;
	SendBuff_USB[usb_send_cnt++]=0xBA;
	SendBuff_USB[usb_send_cnt++]=0xBF;
	SendBuff_USB[usb_send_cnt++]=0x04;
	SendBuff_USB[usb_send_cnt++]=0;

	setDataFloat(1);
	setDataFloat(2);
	setDataFloat(3);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);
	setDataFloat(0);	

	
	setDataFloat(0);
	setDataFloat(0);
	
	setDataFloat(0);	
	setDataFloat(0);
	setDataFloat(0);	
	setDataFloat(0);
		
	SendBuff_USB[cnt_reg+3] =(usb_send_cnt-cnt_reg)-4;
		for( i=cnt_reg;i<usb_send_cnt;i++)
	sum += SendBuff_USB[i];
	SendBuff_USB[usb_send_cnt++] = sum;
}


int cnt_usb_send[5]={0};
void UDP_OCU_TX(void)//发送
{
	static char state=0,i;
	static char state_flag=0,flag1=0;
	int len,t;

	usb_send_cnt=0;
	data_per_usb_robot_state1();
}

int main(int argc, char *argv[])
{
	static int cnt = 0;
	float sys_dt = 0;
	int flag = 0;
    float timer[5]={0};
    int i=0;
   /* sock_fd --- socket文件描述符 创建udp套接字*/  
	int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_fd < 0)  
	{  
		perror("socket");  
		exit(1);  
	}  
   /* 将套接字和IP、端口绑定 */  
   struct sockaddr_in addr_serv;  
   int len;  
   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(SERV_PORT);//端口
   /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */  
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
   len = sizeof(addr_serv);  
     
   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)  
   {  
     perror("bind error:");  
     exit(1);  
   }  
   
    int recv_num=0,send_num=0;  
    char send_buf[500] = "i am server!",recv_buf[500];  
    struct sockaddr_in addr_client;     	
    while (1)
	{
	    sys_dt = Get_Cycle_T(10);
	    timer[0]+=sys_dt;

		//读取客户端
		recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);  

		if(recv_num < 0)  
		{  
			perror("OCU recvfrom error:");  
			exit(1);  
		}  
		//解码
		UDP_RX_PROCESS(recv_buf,recv_num);

		//回传数据
		UDP_OCU_TX();
		for(i=0;i<usb_send_cnt;i++)
			send_buf[i] = SendBuff_USB[i];  

		send_num = sendto(sock_fd, send_buf, usb_send_cnt, 0, (struct sockaddr *)&addr_client, len);  
		//printf("usb_send_cnt=%d\n",send_num);
		if(send_num < 0)  
		{  
			perror("OCU sendto error:");  
			exit(1);  
		}  
        usleep(1*1000);
	}
	close(sock_fd);  
	return 0;
}

