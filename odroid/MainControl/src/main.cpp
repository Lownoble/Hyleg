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
#include "SPI/spi.h"
#include "UART/uart_communicate.h"
#include "FSM/FSM.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include "dataProcess/ShareData.h"

using namespace std;

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


void* Thread_Control(void* arg)//控制管理线程
{
	ShareData* shareData = static_cast<ShareData*>(arg);
	long long startTime;
	/* set real-time process */
    // setProcessScheduler();
    /* set the print format */
	IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
	ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;

	CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002;   //run at 500Hz
	ctrlComp->H = 0.45;
	ctrlComp->Mass_x = 0.00;
    ctrlComp->running = &running;
	ctrlComp->period = 2;
	ctrlComp->stancePhaseRatio = 0.55;
	ctrlComp->robotModel = new HyRobot();
	ctrlComp->waveGen = new WaveGenerator(ctrlComp->period,ctrlComp->stancePhaseRatio, Vec2(0, 0.5));
	ctrlComp->geneObj();
    //ControlFrame ctrlFrame(ctrlComp);
	FSM* _FSMController = new FSM(ctrlComp);
	
    //signal(SIGINT, ShutDown); 
	while (running)
	{
		startTime = getSystemTime();
		shareData->getState(ctrlComp->lowState);
		_FSMController->run();
		shareData->setCmd(ctrlComp->lowCmd);
		absoluteWait(startTime, (long long)(ctrlComp->dt * 1000000));
	}
	delete ctrlComp;
	return 0;
}

void* Thread_SPI(void* arg)//SPI通讯线程
{
	ShareData* shareData = static_cast<ShareData*>(arg);
	long long startTime;
	float cycleTime = 0.005; //200Hz
	SPI* spi = new SPI();
    while (1)
	{
		//-------SPI CAN发送
		startTime = getSystemTime();
		shareData->getCmd(spi);
		spi->transfer(10);
		shareData->setState(spi);
        absoluteWait(startTime, (long long)(cycleTime * 1000000));
	}
    delete spi;

	return 0;
}

void* Thread_UART(void* arg)//UART通讯线程
{
	ShareData* shareData = static_cast<ShareData*>(arg);
	long long startTime;
	float cycleTime = 0.005; //200Hz
	UART* uart = new UART();
    while (1)
	{
		startTime = getSystemTime();
		shareData->getValveCtrl(&(uart->valve));
	    uart->mSerialRead();
        uart->powerSerialRead();
		uart->uartValveCtrl();
		shareData->setSensor(&(uart->sensor));
        absoluteWait(startTime, (long long)(cycleTime * 1000000));
	}
    delete uart;

	return 0;
}



int main(int argc, char *argv[])//目前仅能保证上下通讯频率1ms
{
	ShareData shareData;
    pthread_t tida, tidb, tidc, tidd;
	pthread_create(&tidb, NULL, Thread_Control, &shareData);
	pthread_create(&tidc, NULL, Thread_SPI, &shareData);
	pthread_create(&tidd, NULL, Thread_UART, &shareData);
	pthread_join(tidb, NULL);
	pthread_join(tidc, NULL);
	pthread_join(tidd, NULL);
	return 0;
}

