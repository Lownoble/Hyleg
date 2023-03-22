#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <wiringSerial.h>

#define RX_IN_USB 0

typedef struct {
  uint8_t header[2];
  uint8_t id;
  uint8_t role;
  float pos_3d[3];
  float vel_3d[3];
  float dis_arr[8];
  float imu_gyro_3d[3];
  float imu_acc_3d[3];
  uint8_t reserved1[12];
  float angle_3d[3];
  float quaternion[4];
  uint8_t reserved2[4];
  uint32_t local_time;
  uint32_t system_time;
  uint8_t reserved3[1];
  float eop_3d[3];
  float voltage;
  uint8_t reserved4[5];
  uint8_t check_sum;
}TAG_FRAME0;

TAG_FRAME0 frame0;

using namespace std;
int fd;
#if !RX_IN_USB
serial::Serial m_serial("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(5));
#else
serial::Serial m_serial("/dev/ttyUSB0",115200 , serial::Timeout::simpleTimeout(5));
#endif


void uartSent()
{  unsigned char data_to_send[50];
    int _cnt = 0, i = 0, sum = 0;
    int tx_test[2]={1,2};
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = tx_test[0];

    data_to_send[_cnt++] = int(tx_test[1])>>8;
    data_to_send[_cnt++] = int(tx_test[1])%256;
  
    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    int Length = _cnt;
    serial_port.write(data_to_send, Length);
}


#define BUFF_SIZE 200
unsigned char data_buf[BUFF_SIZE] = {0};

bool checkcommand(const unsigned char *data_buf, int data_length)
{
	uint8_t sum=0;
	int i;
	

	if (!(*(data_buf) == 0x55 && *(data_buf + 1) == 0x01))
	{
		cout<<"Head Check Fail!!!"<<endl;
		return false; 
	}

	for(int32_t i=0;i<data_length-1;++i)
	{
	 sum += *(data_buf+i);
	}

	if (*(data_buf + data_length - 1) != sum)
	{
		cout<<"Sum Check Fail!"<<endl;
		return false;
	}
	return true;
}

uint32_t uint32FromDataf(unsigned char *data,int* anal_cnt)
{
	uint32_t i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return i;
}

uint16_t uint16FromDataf(unsigned char *data,int* anal_cnt)
{
	uint16_t i = 0x00;
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=2;
	return i;
}

float floatFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	float out=0;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	
	 out=(*(float *)&i);
	return out;
}

float int16FromDataf(unsigned char *data,int* anal_cnt)
{
	int16_t temp = (int16_t)((*(data+*anal_cnt+0)) | (*(data+*anal_cnt+1))<< 8 ); 
	float result = temp;
	*anal_cnt +=2;
	return result;
}

int32_t int24FromDataf(unsigned char *data,int* anal_cnt)
{
	int32_t temp = (int32_t)((*(data+*anal_cnt+0)) << 8 | (*(data+*anal_cnt+1))<< 16 | (*(data+*anal_cnt+2))<< 24) / 256; 
	int32_t result = temp;
	*anal_cnt +=3;
	return result;
}

char charFromDataf(unsigned char *data,int* anal_cnt)
{
	char out=0;
	 out=(*(data+*anal_cnt));
	*anal_cnt +=1;
	return (out);
}

char reverseFromDataf(unsigned char *data,int* anal_cnt,int r_num)
{
	*anal_cnt +=r_num;
	return (0);
}

int intFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return (i);
}

void decode(unsigned char *data_buf, int data_length)
{
    int anal_cnt=2;//start num
    frame0.id=charFromDataf(data_buf,&anal_cnt);
    frame0.role=charFromDataf(data_buf,&anal_cnt);

    frame0.pos_3d[0]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
    frame0.pos_3d[1]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
    frame0.pos_3d[2]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;

    frame0.vel_3d[0]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;
    frame0.vel_3d[1]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;
    frame0.vel_3d[2]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;

    frame0.dis_arr[0]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
    frame0.dis_arr[1]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
    frame0.dis_arr[2]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
    frame0.dis_arr[3]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
    frame0.dis_arr[4]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
    frame0.dis_arr[5]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
    frame0.dis_arr[6]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
    frame0.dis_arr[7]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;

    frame0.imu_gyro_3d[0]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.imu_gyro_3d[1]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.imu_gyro_3d[2]=floatFromDataf(data_buf,&anal_cnt);

    frame0.imu_acc_3d[0]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.imu_acc_3d[1]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.imu_acc_3d[2]=floatFromDataf(data_buf,&anal_cnt);

    reverseFromDataf(data_buf,&anal_cnt,12);

    frame0.angle_3d[0]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	
    frame0.angle_3d[1]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	
    frame0.angle_3d[2]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	

    frame0.quaternion[0]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.quaternion[1]=floatFromDataf(data_buf,&anal_cnt);	
    frame0.quaternion[2]=floatFromDataf(data_buf,&anal_cnt);
    frame0.quaternion[3]=floatFromDataf(data_buf,&anal_cnt);

    reverseFromDataf(data_buf,&anal_cnt,4);

    frame0.local_time=uint32FromDataf(data_buf,&anal_cnt);
    frame0.system_time=uint32FromDataf(data_buf,&anal_cnt);

    reverseFromDataf(data_buf,&anal_cnt,1);

    frame0.eop_3d[0]=charFromDataf(data_buf,&anal_cnt)/100.0f;
    frame0.eop_3d[1]=charFromDataf(data_buf,&anal_cnt)/100.0f;
    frame0.eop_3d[2]=charFromDataf(data_buf,&anal_cnt)/100.0f;

    frame0.voltage=uint16FromDataf(data_buf,&anal_cnt)/1000.0f;
}


unsigned char RxBuffer1[200];
unsigned char RxState1 = 0;
int RxBufferNum1 = 0;
int RxBufferCnt1 = 0;
int _data_len1 = 0;
int _data_cnt1 = 0;
void rx_anal(unsigned char com_data)//Rx interupt
{
	if(RxState1==0&&com_data==0x55)
	{
		RxState1=1;
		RxBuffer1[0]=com_data;
	}
	else if(RxState1==1&&com_data==0x01)
	{
		RxState1=2;
		RxBuffer1[1]=com_data;
		_data_len1 = 128 - 2;
		_data_cnt1 = 0;
	}
	else if(RxState1==2&&_data_len1>0)
	{
		_data_len1--;
		RxBuffer1[2+_data_cnt1++]=com_data;
		if(_data_len1==0)
		  RxState1= 3;
	}
	else if(RxState1==3)
	{
		RxState1 = 0;
		RxBuffer1[2+_data_cnt1]=com_data;
		if (_data_cnt1>0)
		{
			if (checkcommand(RxBuffer1, _data_cnt1+2))
			{
				decode(RxBuffer1, _data_cnt1+2);
			}
		}
	}
	else
		RxState1 = 0;
}

int main()
{
int data_length = 0,fd=0,data_cnt=0;
unsigned char rx_temp[200]={0};
#if !RX_IN_USB
    while(( fd = serialOpen ("/dev/ttyS0",115200))<0)
#else
    while(( fd = serialOpen ("/dev/ttyUSB0",115200))<0)
#endif
    {
            cout<<"serial err"<<endl;
            usleep(2000*1000);
    }
	cout<<"serial Open!"<<endl;

    while (1)
    {

        data_length = m_serial.available();
        if (data_length>0)
        {
            cout<<"RX::data_length: "<<data_length<<endl;
            m_serial.read(rx_temp, data_length);     
            for(int i=0;i<data_length;i++)
                rx_anal(rx_temp[i]);
            uartSent();
        }
    }
}
