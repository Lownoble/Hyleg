#ifndef UART_H
#define UART_H



#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "message/LowlevelState.h"

#ifdef __RASPberyPI__  
  // 包含 wiringpi 库的代码  
  #include <wiringpi2/wiringSerial.h> 
#endif

#define RX_IN_USB 0
#define POWER_DATA 1

struct Sensor{
  Treadmile treadmile;
  float pressure;
  float current;

    void GetRecv(LowlevelState *state){
      // state->treadmile = treadmile;
      // printf("speed:%f distance:%f pressure:%f current:%f \n",treadmile.speed,treadmile.distance,pressure,current);
    }
};

extern Sensor sensor;

int uart_init();
void uart_rx(uint8_t *data_buf, int data_length);
void rx_anal(uint8_t *rx_temp, int data_length);
void mSerialRead();
void powerSerialRead();
#endif