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
#include <sys/time.h>
#include <math.h>
#include <time.h> 
#include "comm.h"
#include "spi_node.h"
#include "sys_time.h"
#include "spi.h"

float spi_loss_cnt = 0;
int spi_connect = 0;
pthread_mutex_t lock;
void* Thread_1(void*)//内存管理线程
{
	static int cnt = 0;
	float sys_dt = 0;
	int flag = 0;
    float timer[5]={0};
    int i=0;
 
	while (1)
	{
	    sys_dt = Get_Cycle_T(0);
	    timer[0]+=sys_dt;
		//printf("Thread1!!\n");
        sleep(1);
	}
	return 0;
}

void* Thread_SPI(void*)//SPI通讯线程
{
	static int cnt = 0;
	float sys_dt = 0;
	int flag = 0;
    float timer[5]={0};
    int i=0;
    int memory_update=0;
	int fd = spi_init();
	int MEM_SIZE = 2;
	char buf_mem[MEM_SIZE]={1,2};
  	Cycle_Time_Init();
    while (1)
	{
	    sys_dt = Get_Cycle_T(10);
	    timer[0]+=sys_dt;
		//printf("SPI!!\n");
        spi_loss_cnt += sys_dt;
		if (spi_loss_cnt > 3&& spi_connect==1)
		{
			spi_loss_cnt = 0;
			spi_connect = 0;
			printf("SPI LOSS!!!\n");
		}
		//-------SPI CAN发送
		transfer(fd, 0);
		printf("\n");
        
        sleep(1);
	}
    close(fd);
 
	return 0;
}

int main(int argc, char *argv[])//目前仅能保证上下通讯频率1ms
{

    pthread_t tida, tidb;
	pthread_mutex_init(&lock, NULL);
	pthread_create(&tida, NULL, Thread_1, NULL);
	pthread_create(&tidb, NULL, Thread_SPI, NULL);
	pthread_join(tida, NULL);
	pthread_join(tidb, NULL);

	return 0;
}
