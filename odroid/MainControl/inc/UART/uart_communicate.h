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

#ifdef __RASPberyPI__  
  // 包含 wiringpi 库的代码  
  #include <wiringpi2/wiringSerial.h> 
#endif

#define RX_IN_USB 0

struct TREADMILE{
  float speed;
  float distance;
  float distance0;
  float Kp;
  float Kd;

  TREADMILE() : speed(0.0f), distance(0.0f), distance0(0.0f), Kp(0.0f), Kd(0.0f) {}
};

extern TREADMILE treadmile;

int uart_init();
void uart_rx(uint8_t *data_buf, int data_length);
void rx_anal(int fd);

#endif