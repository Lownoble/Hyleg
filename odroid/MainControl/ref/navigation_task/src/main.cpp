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
#include <pthread.h>
//---
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
		printf("Thread1!!\n");
        sleep(1);
	}
	return 0;
}


void* Thread_2(void*)//OCU UDP通讯线程
{
	static int cnt = 0;
	float sys_dt = 0;
	int flag = 0;
    float timer[5]={0};
    int i=0;
  	
    while (1)
	{
	    sys_dt = Get_Cycle_T(10);
	    timer[0]+=sys_dt;
		printf("Thread2!!\n");
        sleep(1);
	}
 
	return 0;
}

int main(int argc, char *argv[])
{
	pthread_t tida, tidb;
	pthread_mutex_init(&lock, NULL);
	pthread_create(&tida, NULL, Thread_1, NULL);
	pthread_create(&tidb, NULL, Thread_2, NULL);
	pthread_join(tida, NULL);
	pthread_join(tidb, NULL);
	return 1;
}

