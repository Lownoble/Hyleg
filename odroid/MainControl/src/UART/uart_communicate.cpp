#include "UART/uart_communicate.h"

TREADMILE treadmile;

using namespace std;

#if !RX_IN_USB
serial::Serial m_serial("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(5));
#else
serial::Serial m_serial("/dev/ttyUSB0",115200 , serial::Timeout::simpleTimeout(5));
#endif

#define UART_BUF_SIZE 200
uint8_t uart_rx_buf[UART_BUF_SIZE] = {0};

int uart_init()
{
	m_serial.close();
	int fd;
	m_serial.open ();
	if (m_serial.isOpen () < 0){
		cout<<"serial err"<<endl;
		usleep(2000*1000);
	}
	else	
		cout<<"serial Open!"<<endl;
		
	return fd;
}


void uartSent()
{  
	unsigned char data_to_send[50];
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
    m_serial.write(data_to_send, Length);
}


uint32_t intFromData(unsigned char *data,int* anal_cnt)
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

float floatFromData(unsigned char *data,int* anal_cnt)
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


void rx_anal(int fd)//Rx interupt
{
	static int RxState = 0;
	static int _data_len1 = 0;
	static int _data_cnt1 = 0;
	int data_length = 0;
	uint8_t rx_temp[UART_BUF_SIZE]={0};
	uint8_t data;
	data_length = m_serial.available();
        if (data_length>0)
        {
            // cout<<"RX::data_length: "<<data_length<<endl;
            m_serial.read(rx_temp, data_length);
			// for(int i = 0; i < data_length; i++)
			// {
			// 	printf("%02x ",rx_temp[i]);
			// }
			// printf("\n");
            for(int i = 0; i < data_length; i++)
			{
				data = rx_temp[i];
				if(RxState == 0 && data == 0xFF)
				{
					RxState = 1;
					uart_rx_buf[0] = data;
				}
				else if(RxState == 1 && data == 0xFC)
				{
					RxState = 2;
					uart_rx_buf[1] = data;
				}
				else if (RxState == 2 && data >= 0 && data < 0XF1)
				{
					RxState = 3;
					uart_rx_buf[2] = data;
					_data_len1 = 128 - 2;
					_data_cnt1 = 0;
				}
				else if(RxState == 3 && data < UART_BUF_SIZE)
				{
					RxState= 4;
					uart_rx_buf[3] = data;
					_data_len1 = data;
					_data_cnt1 = 0; 
				}
				else if (RxState == 4 && _data_len1 > 0)
				{
					_data_len1--;
					uart_rx_buf[4 + _data_cnt1++] = data;
					if (_data_len1 == 0)
						RxState = 5;
				}
				else if(RxState == 5)
				{
					RxState = 0;
					uart_rx_buf[4 + _data_cnt1] = data;
					if (_data_cnt1>0)
						uart_rx(uart_rx_buf, _data_cnt1+5);
				}
				else
					RxState = 0;
        }
	}
}

void uart_rx(uint8_t *data_buf, int data_length)
{
	uint8_t sum=0;
	uint8_t sel = *(data_buf + 2);
	int anal_cnt = 4;
	if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFC))
	{
		cout<<"Head Check Fail!!!"<<endl;
		return; 
	}

	for(int i=0; i<data_length-1; i++)
		sum += *(data_buf+i);

	if (*(data_buf + data_length - 1) != sum)
	{
		cout<<"Sum Check Fail!"<<endl;
		return;
	}

	switch(sel)
	{
	case 0:

		printf("000 ");
		break;
	case 5:
		treadmile.speed = floatFromData(data_buf, &anal_cnt);
		// printf("speed: %f\n",treadmile.speed);
		break;
	case 6:
		treadmile.distance = floatFromData(data_buf, &anal_cnt);
		// printf("distance: %f\n",treadmile.distance);
		break;

	default:
		printf("not define sel\n");
		break;
	}

	return;
}
