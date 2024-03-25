#include "UART/uart_communicate.h"
using namespace std;

#define RX_IN_USB 1
#define POWER_DATA 1

//ls -l /dev/tty*
#if !RX_IN_USB
serial::Serial m_serial("/dev/ttyS0",115200 , serial::Timeout::simpleTimeout(5));
#else
serial::Serial m_serial("/dev/Sensor",460800 , serial::Timeout::simpleTimeout(5));

#endif

#if POWER_DATA
	serial::Serial power_serial("/dev/Lower",576000 , serial::Timeout::simpleTimeout(5));
#endif

UART::UART(){
	_fd = uart_init();
}

UART::~UART(){
	close(_fd);
}

int UART::uart_init()
{
	m_serial.close();
	int fd;
	m_serial.open ();
	if (m_serial.isOpen () < 0){
		cout<<"my serial err"<<endl;
		usleep(2000*1000);
	}
	else	
		cout<<"my serial Open!"<<endl;

	#if POWER_DATA
	power_serial.close();
	power_serial.open ();
	if (power_serial.isOpen () < 0){
		cout<<"power serial err"<<endl;
		usleep(2000*1000);
	}
	else	
		cout<<"power serial Open!"<<endl;
	#endif

	return fd;
}


void UART::uartValveCtrl(){
	VecInt2 valveSignal = valve.valveSignal;
	static VecInt2 valveSignalPast;
	unsigned char data_to_send[50];
    int _cnt = 0;
	if(valveSignal(0)&&valveSignal(1)){ 			// 左右都高压
		for(int i=0; i<1; i++){
		data_to_send[_cnt++] = 0x0A;
		data_to_send[_cnt++] = 0x0B;
		data_to_send[_cnt++] = 0x00;
		data_to_send[_cnt++] = 0x00;
		data_to_send[_cnt++] = 0x15;
		}
	}
	else if(!valveSignal(0)&&!valveSignal(1)){		// 左右都低压
		for(int i=0; i<1; i++){
		data_to_send[_cnt++] = 0x0A;
		data_to_send[_cnt++] = 0x0B;
		data_to_send[_cnt++] = 0xFF;
		data_to_send[_cnt++] = 0xFF;
		data_to_send[_cnt++] = 0x13;
		}
	}
	else if(valveSignal(0)&&!valveSignal(1)){		// 左高压，右低压
		for(int i=0; i<1; i++){
		data_to_send[_cnt++] = 0x0A;
		data_to_send[_cnt++] = 0x0B;
		data_to_send[_cnt++] = 0xFF;
		data_to_send[_cnt++] = 0x00;
		data_to_send[_cnt++] = 0x14;
		}
	}
	else if(!valveSignal(0)&&valveSignal(1)){		// 右高压，左低压
		for(int i=0; i<1; i++){
		data_to_send[_cnt++] = 0x0A;
		data_to_send[_cnt++] = 0x0B;
		data_to_send[_cnt++] = 0x00;
		data_to_send[_cnt++] = 0xFF;
		data_to_send[_cnt++] = 0x14;
		}
	}
	// if(valveSignal!=valveSignalPast){
		m_serial.write(data_to_send, _cnt);
		// printf("%d %d\n",valveSignal(0),valveSignal(1));
		// for(int i=0;i<_cnt;i++){
		// 	printf("%02x ",data_to_send[i]);
		// }
		// cout <<endl;
	// }
	valveSignalPast = valveSignal;
}


uint32_t UART::intFromData(unsigned char *data,int* anal_cnt)
{
	uint32_t i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return i;
}

uint16_t UART::uint16FromDataf(unsigned char *data,int* anal_cnt)
{
	uint16_t i = 0x00;
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=2;
	return i;
}

float UART::floatFromData(unsigned char *data,int* anal_cnt)
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

float UART::int16FromDataf(unsigned char *data,int* anal_cnt)
{
	int16_t temp = (int16_t)((*(data+*anal_cnt+0)) | (*(data+*anal_cnt+1))<< 8 ); 
	float result = temp;
	*anal_cnt +=2;
	return result;
}

int32_t UART::int24FromDataf(unsigned char *data,int* anal_cnt)
{
	int32_t temp = (int32_t)((*(data+*anal_cnt+0)) << 8 | (*(data+*anal_cnt+1))<< 16 | (*(data+*anal_cnt+2))<< 24) / 256; 
	int32_t result = temp;
	*anal_cnt +=3;
	return result;
}

char UART::charFromDataf(unsigned char *data,int* anal_cnt)
{
	char out=0;
	 out=(*(data+*anal_cnt));
	*anal_cnt +=1;
	return (out);
}

char UART::reverseFromDataf(unsigned char *data,int* anal_cnt,int r_num)
{
	*anal_cnt +=r_num;
	return (0);
}

int UART::intFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return (i);
}

void UART::mSerialRead(){
	int data_length = 0;
	uint8_t rx_temp[UART_BUF_SIZE]={0};
	data_length = m_serial.available();
	if (data_length>0)
    {
		m_serial.read(rx_temp, data_length);
		rx_anal(rx_temp,data_length);
	}
}


void UART::powerSerialRead(){
#if POWER_DATA
	int data_length = 0;
	uint8_t rx_temp[UART_BUF_SIZE]={0};
	data_length = power_serial.available();
	if (data_length>0)
    {
		power_serial.read(rx_temp, data_length);
		rx_anal(rx_temp,data_length);
	}
#endif
}



void UART::rx_anal(uint8_t *rx_temp, int data_length)//Rx interupt
{
	static int RxState = 0;
	static int _data_len1 = 0;
	static int _data_cnt1 = 0;
	uint8_t data;
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

void UART::uart_rx(uint8_t *data_buf, int data_length)
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
		// for(int i = 0; i < data_length; i++)
		// {
		// 	printf("%02x ",*(data_buf+i));
		// }
		printf("\n");
		return;
	}

	switch(sel)
	{
	case 0:
		printf("000 ");
		break;
	case 5:
		sensor.treadmile.speed = floatFromData(data_buf, &anal_cnt);
		// printf("speed: %f\n",sensor.treadmile.speed);
		break;
	case 6:
		sensor.treadmile.distance = floatFromData(data_buf, &anal_cnt);
		// printf("distance: %f\n",sensor.treadmile.distance);
		break;
	case 7:
		sensor.pressure = floatFromData(data_buf, &anal_cnt);
		printf("pressure: %f\n",sensor.pressure);
		break;	
	case 8:
		sensor.treadmile.speed = floatFromData(data_buf, &anal_cnt);
		sensor.treadmile.distance = floatFromData(data_buf, &anal_cnt)/100;
		// printf("speed: %f distance: %f\n",sensor.treadmile.speed,sensor.treadmile.distance);
		break;
	case 9:
		sensor.current = floatFromData(data_buf, &anal_cnt);
		sensor.pressure = floatFromData(data_buf, &anal_cnt);
		// printf("current: %f pressure: %f\n",sensor.current,sensor.pressure);
		break;

	default:
		printf("not define sel\n");
		break;
	}

	return;
}
