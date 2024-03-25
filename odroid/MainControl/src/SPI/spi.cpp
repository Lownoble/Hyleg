#include "SPI/spi.h"
using namespace std;


SPI::SPI(){
	for(int i=0;i<5;i++){
        motorCmd[i].ID = i;
        motorState[i].ID = i;
    }
	footContact.setZero();
	_fd = spi_init();
}

SPI::~SPI(){
	close(_fd);
}


int SPI::spi_init()
{
	static const char *device = "/dev/spidev0.0";
	int ret = 0;
	int fd;
	fd = open(device, O_RDWR);
	if (fd < 0)
		cout << "can't open device" <<endl;
	//设置模式
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode); 
	if (ret == -1)
		cout << "can't set spi mode" <<endl;
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode); 
	if (ret == -1)
		cout << "can't get spi mode" <<endl;
	//设置一次多少位
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits); 
	if (ret == -1)
		cout << "can't set bits per word" <<endl;
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits); 
	if (ret == -1)
		cout << "can't get bits per word" <<endl;
	//设置最大速度
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); 
	if (ret == -1)
		cout << "can't set max speed hz" <<endl;
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed); 
	if (ret == -1)
		cout << "can't get max speed hz" <<endl;

    printf("spi fd:%d\n", fd);
	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);

	return fd;
}

void SPI::setDataInt_spi(int i)
{
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

void SPI::setDataFloat_spi(float f)
{
	int i = *(int *)&f;
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

float SPI::floatFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));

	*anal_cnt += 4;
	return *(float *)&i;
}

char SPI::charFromData_spi(unsigned char *data, int *anal_cnt)
{
	int temp = *anal_cnt;
	*anal_cnt += 1;
	return *(data + temp);
}

int SPI::intFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));
	*anal_cnt += 4;
	return i;
}


void SPI::can_board_send(int sel)//发送到单片机
{
	int i;
	static float t_temp = 0;
	char id = 0;
	char sum_t = 0, _cnt = 0;
	spi_tx_cnt = 0;

	spi_tx_buf[spi_tx_cnt++] = 0xFA;
	spi_tx_buf[spi_tx_cnt++] = 0xFF;
	spi_tx_buf[spi_tx_cnt++] = sel;
	spi_tx_buf[spi_tx_cnt++] = 0;

	switch (sel)
	{
	case 0:
		setDataInt_spi(0);
		setDataFloat_spi(3.14);
		setDataFloat_spi(2.22);
		setDataFloat_spi(38.15);
		setDataFloat_spi(38.15);
		setDataFloat_spi(38.15);
		break;
	case 1:
	case 2:
	case 3:
	case 4://发送电流力矩  
		setDataInt_spi(motorCmd[sel].ID);
		setDataFloat_spi(motorCmd[sel].q);
		setDataFloat_spi(motorCmd[sel].dq);
		setDataFloat_spi(motorCmd[sel].tau);
		setDataFloat_spi(motorCmd[sel].Kp);
		setDataFloat_spi(motorCmd[sel].Kd);
		// printf("%f ",motorCmd[sel].q);
		break;

	case 10:
		for(int i=1; i<5; i++){
			setDataInt_spi(motorCmd[i].ID);
			setDataInt_spi(motorCmd[i].mode);
			setDataFloat_spi(motorCmd[i].q);
			setDataFloat_spi(motorCmd[i].dq);
			setDataFloat_spi(motorCmd[i].tau);
			setDataFloat_spi(motorCmd[i].Kp);
			setDataFloat_spi(motorCmd[i].Kd);
			// printf("%f ",motorCmd[i].q);
		}
		setDataFloat_spi(current);
		setDataFloat_spi(pressure);
		setDataFloat_spi(velocity);
		// printf("%f %f \n",current,pressure);
		// printf("\n");
		break;

	case 98:// FA FF 62 08 0F 01 00 00 9C 00 00 00 0F 00 00
		setDataInt_spi(271); 
		setDataInt_spi(156); 
		break;
	case 99:
		t_temp += 0.001 * 0.1;
		setDataFloat_spi(2 * sin(t_temp));
		setDataFloat_spi(-2 * sin(t_temp));
		break;
	default:
		for (int id = 0; id < 4; id++)
		{
			setDataFloat_spi(0); 
			setDataFloat_spi(0);
			setDataFloat_spi(0);
			setDataFloat_spi(0);
		}
		break;
	}

	spi_tx_buf[3] = (spi_tx_cnt)-4;
	for (i = 0; i < spi_tx_cnt; i++)
		sum_t += spi_tx_buf[i];
	spi_tx_buf[spi_tx_cnt++] = sum_t;

	for(int i=0;i<3;i++){
		spi_tx_buf[spi_tx_cnt++] = 0;
	}

}


void SPI::slave_rx(uint8_t *data_buf, int num)//接收解码
{
	static int cnt_p = 0;
	uint8_t id;
	uint8_t sum = 0;
	uint8_t i;
	uint8_t temp;
	uint8_t sel = *(data_buf + 2);
	int anal_cnt = 4;

	int ID;
	float angle;
	float speed;
	float current;
	int rx_test[2]={0};
	for (i = 0; i < (num - 1); i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1)))
		return; 
	if (!(*(data_buf) == 0xFF && *(data_buf + 1) == 0xFB))
		return;				

	switch(sel)
	{
	case 0:
		ID = intFromData_spi(data_buf, &anal_cnt);
		angle = floatFromData_spi(data_buf, &anal_cnt);
		speed = floatFromData_spi(data_buf, &anal_cnt);
		current = floatFromData_spi(data_buf, &anal_cnt);
		printf("%d  %f  %f  %f\n",ID,angle,speed,current);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		ID = intFromData_spi(data_buf,&anal_cnt);
		motorState[ID].q = floatFromData_spi(data_buf,&anal_cnt);
		motorState[ID].dq = floatFromData_spi(data_buf,&anal_cnt);
		motorState[ID].ddq = floatFromData_spi(data_buf,&anal_cnt);
		motorState[ID].tauEst = floatFromData_spi(data_buf,&anal_cnt);
		motorState[ID].mode = floatFromData_spi(data_buf,&anal_cnt);
		//printf("%d %f %f %f %f\n",ID,motorState[ID].q,motorState[ID].dq,motorState[ID].ddq,motorState[ID].tauEst);
		// if(ID==3) printf("%d %f %f %f %f  ",ID,motorState[ID].q,motorState[ID].dq,motorState[ID].ddq,motorState[ID].tauEst);
		// if(ID==4) printf("%d %f %f %f %f\n",ID,motorState[ID].q,motorState[ID].dq,motorState[ID].ddq,motorState[ID].tauEst);
		break;

	case 10:
		for(int i=1;i<5;i++){
			ID = intFromData_spi(data_buf,&anal_cnt);
			motorState[ID].q = floatFromData_spi(data_buf,&anal_cnt);
			motorState[ID].dq = floatFromData_spi(data_buf,&anal_cnt);
			motorState[ID].ddq = floatFromData_spi(data_buf,&anal_cnt);
			motorState[ID].tauEst = floatFromData_spi(data_buf,&anal_cnt);
			motorState[ID].mode = floatFromData_spi(data_buf,&anal_cnt);
			motorState[ID].mode = floatFromData_spi(data_buf,&anal_cnt);
			// printf("%d %f %f %f %f  ",ID,motorState[ID].q,motorState[ID].dq,motorState[ID].ddq,motorState[ID].tauEst);
		}
		footContact(0) = intFromData_spi(data_buf,&anal_cnt);
		footContact(1) = intFromData_spi(data_buf,&anal_cnt);
		//printf("%d %d\n",footContact(0),footContact(1));
		break;
	case 98:
		rx_test[0] = intFromData_spi(spi_rx_buf, &anal_cnt);
		rx_test[1] = intFromData_spi(spi_rx_buf, &anal_cnt);
		printf("spi_rx:: %d %d\n",rx_test[0] ,rx_test[1] );
		break;
	default:
		//printf("not define sel\n");
		break;
	}

}

void SPI::transfer(int sel)//发送
{
	static uint8_t state, rx_cnt;
	static uint8_t _data_len2 = 0, _data_cnt2 = 0;
	int ret;
	uint8_t data = 0;

	can_board_send(sel);
	char rx[ARRAY_SIZE(spi_tx_buf)] = {};

	struct spi_ioc_transfer tr;
	memset(&tr, 0, sizeof(tr));
	tr = {
		.tx_buf = (unsigned long)spi_tx_buf,
		.rx_buf = (unsigned long)rx,
		.len = (unsigned int)spi_tx_cnt,
		.speed_hz = speed,
		.delay_usecs = delay,
		.bits_per_word = bits,
		.cs_change = cs,
	};
	ret = ioctl(_fd, SPI_IOC_MESSAGE(1), &tr); 
	if (ret < 1)
		cout << "can't send spi message" <<endl;
	else
	{
		for (int i = 0; i < spi_tx_cnt; i++)
		{
			data = rx[i];
			if (state == 0 && data == 0xFF)
			{
				//printf("1");
				state = 1;
				spi_rx_buf[0] = data;
			}
			else if (state == 1 && data == 0xFB)
			{
				//printf("2");
				state = 2;
				spi_rx_buf[1] = data;
			}
			else if (state == 2 && data >= 0 && data < 0XF1)
			{
				//printf("3");
				state = 3;
				spi_rx_buf[2] = data;
			}
			else if (state == 3 && data < SPI_BUF_SIZE)
			{
				//printf("4");
				state = 4;
				spi_rx_buf[3] = data;
				_data_len2 = data;
				_data_cnt2 = 0;                     
			}
			else if (state == 4 && _data_len2 > 0)
			{
				_data_len2--;
				spi_rx_buf[4 + _data_cnt2++] = data;
				if (_data_len2 == 0) 
					state = 5;
			}
			else if (state == 5)
			{
				//printf("5");
				state = 0;
				spi_rx_buf[4 + _data_cnt2] = data;
				slave_rx(spi_rx_buf, _data_cnt2 + 5);
			}
			else
			{
				//if(state ==1) i = i-1;
				state = 0;
			}
		}
	}
}