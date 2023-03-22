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
#include <time.h> 
#include "comm.h"
#include "spi_node.h"
#include "sys_time.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MEM_SPI 1111

static void transfer(int fd);
static void pabort(const char *s);
static const char *device = "/dev/spidev0.0";

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 1500000 / 2;
static uint16_t delay = 0;
static uint8_t cs = 1;

float spi_loss_cnt = 0;
int spi_connect = 0;

using namespace std;

_SPI_RX spi_rx;
_SPI_TX spi_tx;

uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
int spi_tx_cnt = 0;
int spi_rx_cnt = 0;

char tx[SPI_BUF_SIZE] = {};
char rx[ARRAY_SIZE(tx)] = {};


static void pabort(const char *s)
{
	perror(s);
	abort();
}

#define MEM_SIZE 2048  

int mem_connect=0;
float mem_loss_cnt=0;
struct shareMemory 
{  
    int  flag=0;  //作为一个标志，非0：表示可读，0表示可写
    unsigned char szMsg[MEM_SIZE];  
};  
struct shareMemory shareMemory_spi;

unsigned char mem_write_buf[MEM_SIZE];  
unsigned char mem_read_buf[MEM_SIZE];  
int mem_write_cnt=0;
static void setDataFloat_mem(float f)
{
	int i = *(int *)&f;
	mem_write_buf[mem_write_cnt++] = ((i << 24) >> 24);
	mem_write_buf[mem_write_cnt++] = ((i << 16) >> 24);
	mem_write_buf[mem_write_cnt++] = ((i << 8) >> 24);
	mem_write_buf[mem_write_cnt++] = (i >> 24);
}

void memory_write(void)//写入内存 
{
	static float temp=0;
	mem_write_cnt=0;
	setDataFloat_mem( 1);
	setDataFloat_mem( 2);
}

float rx_test[2]={0};
void memory_read(void){//读取内存
int mem_read_cnt=MEM_SIZE/2;
float test1,test2;

    rx_test[0] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
    rx_test[1] = floatFromData_spi(mem_read_buf, &mem_read_cnt);
	printf("rx:: %f %f\n",rx_test[0] ,rx_test[1] );
}

int main(int argc, char *argv[])//目前仅能保证上下通讯频率1ms
{
	static int cnt = 0;
	float sys_dt = 0;
	int i=0,memory_update=0;
	int flag = 0;
	int fd = spi_init();
	char buf_mem[MEM_SIZE]={1,2};
	Cycle_Time_Init();
	//共享内存
    int shmid_rx = shmget((key_t)MEM_SPI, sizeof(shareMemory_spi), 0666|IPC_CREAT); //失败返回-1，假设成功。
    //0666表示权限，与文件一样。如0644,它表示允许一个进程创建的共享内存被内存创建者所拥有的进程向共享内存读取和写入数据，同时其他用户创建的进程只能读取共享内存。
    void *shm_rx = shmat(shmid_rx, (void*)0, 0);  //失败返回-1，假设成功 
    shareMemory *pshm_rx = (shareMemory*)shm_rx;  
    pshm_rx->flag = 0; 
    printf("Memory SPI attached at %p\n",shm_rx);

	while (1)
	{
		sys_dt = Get_Cycle_T(0);
		//共享内存写
        if(pshm_rx->flag == 0)  
        {  
			if(!mem_connect){
			printf("MEM Control CONNECT!!!\n");
			mem_connect=1;
			}
			mem_loss_cnt=0;
			memory_write();
			for(int k=0;k<MEM_SIZE/2-1;k++)
				pshm_rx->szMsg[k]=mem_write_buf[k];	
            for(int k=MEM_SIZE/2;k<MEM_SIZE;k++)
				mem_read_buf[k]=pshm_rx->szMsg[k];
			memory_read();
            pshm_rx->flag = 1;  
		}else
			mem_loss_cnt+=sys_dt;

		if(mem_loss_cnt>1&&mem_connect==1){
			mem_connect=0;
			mem_loss_cnt=0;
			for (int i = 0; i < 4; i++){
			spi_tx.q_set[i][0] = spi_rx.q[i][0];
			spi_tx.q_set[i][1] = spi_rx.q[i][1];
			spi_tx.tau_ff[i][0] = 0;
			spi_tx.tau_ff[i][1] = 0;
			}
			spi_tx.kp= spi_tx.ki= spi_tx.kd= spi_tx.en_motor= 0;

			printf("MEM Control LOSS!!!\n");
		}	
	}
    shmdt(shm_rx);  //失败返回-1，假设成功 
    shmctl(shmid_rx, IPC_RMID, 0);  //
	close(fd);
	return 0;
}
