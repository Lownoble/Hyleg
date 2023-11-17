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
#include <csignal>
#include <sched.h>
//#include "SPI/comm.h"
#include "SPI/spi_node.h"
#include "SPI/sys_time.h"
#include "SPI/spi.h"
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"

pthread_mutex_t lock;

bool running = true;
//over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

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


void* Thread_Control(void*)//控制管理线程
{
	float sys_dt = 0;
    float timer[5]={0};

	/* set real-time process */
    setProcessScheduler();
    /* set the print format */
	IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
	ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;

	CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002;   //run at 500Hz
    ctrlComp->running = &running;
	ctrlComp->robotModel = new HyRobot();
	ctrlComp->waveGen = new WaveGenerator(2.0,0.6, Vec2(0, 0.5));
	ctrlComp->geneObj();
    ControlFrame ctrlFrame(ctrlComp);
	

    //signal(SIGINT, ShutDown); 
	while (running)
	{
		ctrlFrame.run();
	}
	delete ctrlComp;
	return 0;
}

void* Thread_SPI(void*)//SPI通讯线程
{
	float spi_loss_cnt = 0;
	int spi_connect = 0;	
	static int cnt = 0;
	float sys_dt = 0;
	int flag = 0;
	long long startTime;
	float cycleTime = 0.025; //40Hz
    float timer[5]={0};
    int i=0;
    int memory_update=0;
	int fd = spi_init();
  	Cycle_Time_Init();
    while (1)
	{
	    // sys_dt = Get_Cycle_T(10);
	    // timer[0]+=sys_dt;
        // spi_loss_cnt += sys_dt;
		// if (spi_loss_cnt > 3&& spi_connect==1)
		// {
		// 	spi_loss_cnt = 0;
		// 	spi_connect = 0;
		// 	printf("SPI LOSS!!!\n");
		// }
		//-------SPI CAN发送
		startTime = getSystemTime();
		for(int i=1;i<5;i++){
			transfer(fd, i);
		}
		// printf("\n");
        
        absoluteWait(startTime, (long long)(cycleTime * 1000000));
	}
    close(fd);

	return 0;
}

int main(int argc, char *argv[])//目前仅能保证上下通讯频率1ms
{
    pthread_t tida, tidb, tidc;
	pthread_mutex_init(&lock, NULL);
	pthread_create(&tida, NULL, Thread_1, NULL);
	pthread_create(&tidb, NULL, Thread_Control, NULL);
	pthread_create(&tidc, NULL, Thread_SPI, NULL);
	pthread_join(tida, NULL);
	pthread_join(tidb, NULL);
	pthread_join(tidc, NULL);

}
