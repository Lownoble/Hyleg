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
#include "SPI/spi_node.h"
#include "SPI/spi.h"
#include "UART/uart_communicate.h"
#include "FSM/FSM.h"
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
	ctrlComp->H = 0.450;
	ctrlComp->Mass_x = 0.00;
    ctrlComp->running = &running;
	ctrlComp->robotModel = new HyRobot();
	ctrlComp->waveGen = new WaveGenerator(2.0,0.6, Vec2(0, 0.5));
	ctrlComp->geneObj();
    //ControlFrame ctrlFrame(ctrlComp);
	FSM* _FSMController = new FSM(ctrlComp);
	

    //signal(SIGINT, ShutDown); 
	while (running)
	{
		_FSMController->run();
	}
	delete ctrlComp;
	return 0;
}

void* Thread_SPI(void*)//SPI通讯线程
{
	long long startTime;
	float cycleTime = 0.005; //200Hz
	int fd = spi_init();
  	//Cycle_Time_Init();
    while (1)
	{
		//-------SPI CAN发送
		startTime = getSystemTime();
		transfer(fd, 10);
        
        absoluteWait(startTime, (long long)(cycleTime * 1000000));
	}
    close(fd);

	return 0;
}

void* Thread_UART(void*)//UART通讯线程
{
	long long startTime;
	float cycleTime = 0.002; //10Hz
	int fd = uart_init();
  	//Cycle_Time_Init();
    while (1)
	{
		startTime = getSystemTime();
	    rx_anal(fd);
        
        absoluteWait(startTime, (long long)(cycleTime * 1000000));
	}
    close(fd);

	return 0;
}

int main(int argc, char *argv[])//目前仅能保证上下通讯频率1ms
{
    pthread_t tida, tidb, tidc, tidd;
	pthread_mutex_init(&lock, NULL);
	pthread_create(&tidb, NULL, Thread_Control, NULL);
	pthread_create(&tidc, NULL, Thread_SPI, NULL);
	// pthread_create(&tidd, NULL, Thread_UART, NULL);
	pthread_join(tidb, NULL);
	pthread_join(tidc, NULL);
	// pthread_join(tidd, NULL);
	return 0;
}
