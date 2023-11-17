#include "SPI/spi.h"

SPI _spi;
//_spi.SPIInit();

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 1111

static void transfer(int fd);
static void pabort(const char *s);
static const char *device = "/dev/spidev0.0";

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 50000;
static uint16_t delay = 0;
static uint8_t cs = 0;

using namespace std;

_SPI_RX spi_rx;
_SPI_TX spi_tx;

uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;
int extra_len[2] = {3,3};
int rx_wrong = 0;

char tx[SPI_BUF_SIZE] = {};
char rx[ARRAY_SIZE(tx)] = {};


static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void setDataInt_spi(int i)
{
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
	int i = *(int *)&f;
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static float floatFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));

	*anal_cnt += 4;
	return *(float *)&i;
}

static char charFromData_spi(unsigned char *data, int *anal_cnt)
{
	int temp = *anal_cnt;
	*anal_cnt += 1;
	return *(data + temp);
}

static int intFromData_spi(unsigned char *data, int *anal_cnt)
{
	int i = 0x00;
	i |= (*(data + *anal_cnt + 3) << 24);
	i |= (*(data + *anal_cnt + 2) << 16);
	i |= (*(data + *anal_cnt + 1) << 8);
	i |= (*(data + *anal_cnt + 0));
	*anal_cnt += 4;
	return i;
}

int spi_init()
{
	int ret = 0;
	int fd;
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	//设置模式
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode); 
	if (ret == -1)
		pabort("can't set spi mode");
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode); 
	if (ret == -1)
		pabort("can't get spi mode");
	//设置一次多少位
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits); 
	if (ret == -1)
		pabort("can't set bits per word");
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits); 
	if (ret == -1)
		pabort("can't get bits per word");
	//设置最大速度
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); 
	if (ret == -1)
		pabort("can't set max speed hz");
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed); 
	if (ret == -1)
		pabort("can't get max speed hz");

    printf("spi fd:%d\n", fd);
	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed / 1000);

	return fd;
}

void can_board_send(int sel)//发送到单片机
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
		break;
	case 1:
	case 2:
	case 3:
	case 4://发送电流力矩  
		setDataInt_spi(_spi.motorCmd[sel].ID);
		setDataFloat_spi(_spi.motorCmd[sel].q);
		setDataFloat_spi(_spi.motorCmd[sel].dq);
		setDataFloat_spi(_spi.motorCmd[sel].tau);
		setDataFloat_spi(_spi.motorCmd[sel].Kp);
		setDataFloat_spi(_spi.motorCmd[sel].Kd);
		// printf("%f ",_spi.motorCmd[sel].q);
		break;

	case 10:
		for(int i=1; i<5; i++){
			setDataInt_spi(_spi.motorCmd[i].ID);
			setDataFloat_spi(_spi.motorCmd[i].q);
			setDataFloat_spi(_spi.motorCmd[i].dq);
			setDataFloat_spi(_spi.motorCmd[i].tau);
			setDataFloat_spi(_spi.motorCmd[i].Kp);
			setDataFloat_spi(_spi.motorCmd[i].Kd);
		}
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

	for(int i=0;i<extra_len[rx_wrong];i++){
		spi_tx_buf[spi_tx_cnt++] = 0;
	}

}


void slave_rx(uint8_t *data_buf, int num)//接收解码
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
		_spi.motorState[ID].q = floatFromData_spi(data_buf,&anal_cnt);
		_spi.motorState[ID].dq = floatFromData_spi(data_buf,&anal_cnt);
		_spi.motorState[ID].ddq = floatFromData_spi(data_buf,&anal_cnt);
		_spi.motorState[ID].current = floatFromData_spi(data_buf,&anal_cnt);
		_spi.motorState[ID].none = floatFromData_spi(data_buf,&anal_cnt);
		//printf("%d %f %f %f %f\n",ID,_spi.motorState[ID].q,_spi.motorState[ID].dq,_spi.motorState[ID].ddq,_spi.motorState[ID].current);
		// if(ID==3) printf("%d %f %f %f %f  ",ID,_spi.motorState[ID].q,_spi.motorState[ID].dq,_spi.motorState[ID].ddq,_spi.motorState[ID].current);
		// if(ID==4) printf("%d %f %f %f %f\n",ID,_spi.motorState[ID].q,_spi.motorState[ID].dq,_spi.motorState[ID].ddq,_spi.motorState[ID].current);
		break;

	case 10:
		for(int i=1;i<5;i++){
			ID = intFromData_spi(data_buf,&anal_cnt);
			_spi.motorState[ID].q = floatFromData_spi(data_buf,&anal_cnt);
			_spi.motorState[ID].dq = floatFromData_spi(data_buf,&anal_cnt);
			_spi.motorState[ID].ddq = floatFromData_spi(data_buf,&anal_cnt);
			_spi.motorState[ID].current = floatFromData_spi(data_buf,&anal_cnt);
			_spi.motorState[ID].none = floatFromData_spi(data_buf,&anal_cnt);
			// printf("%d %f %f %f %f  ",ID,_spi.motorState[ID].q,_spi.motorState[ID].dq,_spi.motorState[ID].ddq,_spi.motorState[ID].current);
		}
		// printf("\n");
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

void transfer(int fd, int sel)//发送
{
	static uint8_t state, rx_cnt;
	static uint8_t _data_len2 = 0, _data_cnt2 = 0;
	int ret;
	uint8_t data = 0;

	can_board_send(sel);

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

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr); 

	if (ret < 1)
		pabort("can't send spi message");
	else
	{
		if(rx[0]!=0xff && rx_wrong==0){
			int i = 0;
			rx_wrong = 1;
			while(rx[i] != 0xff)
			{
				i++;
			}
			extra_len[1] = 3+i;
		}
		else rx_wrong = 0;

		// printf("RX:");
		// for (int i = 0; i < spi_tx_cnt; i++)	printf("%02x ",rx[i]);
		// printf("\nTX:");
		// for (int i = 0; i < spi_tx_cnt; i++) printf("%02x ",spi_tx_buf[i]);
		// printf("\n\n");

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
				spi_rx_cnt = 4;
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