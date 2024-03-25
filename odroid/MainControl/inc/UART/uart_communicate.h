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
#include "message/LowlevelCmd.h"

#ifdef __RASPberyPI__  
  // 包含 wiringpi 库的代码  
  #include <wiringpi2/wiringSerial.h> 
#endif

#define UART_BUF_SIZE 1024

struct Sensor{
  Treadmile treadmile;
  float pressure;
  float current;

  void GetRecv(LowlevelState *state){
    state->treadmile = treadmile;
  }
};

struct Valve{
  VecInt2 valveSignal;
  void SetSend(const LowlevelCmd *cmd){
    valveSignal = cmd->valveSignal;
  }
};

class UART{
public:
  UART();
  ~UART();

  Sensor sensor;
  Valve valve;

  void mSerialRead();
  void powerSerialRead();
  void uartValveCtrl();
private:
  int _fd;
  uint8_t uart_rx_buf[UART_BUF_SIZE] = {0};
  uint8_t uart_tx_buf[UART_BUF_SIZE] = {0};

  int uart_init();
  void uart_rx(uint8_t *data_buf, int data_length);
  void rx_anal(uint8_t *rx_temp, int data_length);

  uint32_t intFromData(unsigned char *data,int* anal_cnt);
  uint16_t uint16FromDataf(unsigned char *data,int* anal_cnt);
  float floatFromData(unsigned char *data,int* anal_cnt);
  float int16FromDataf(unsigned char *data,int* anal_cnt);
  int32_t int24FromDataf(unsigned char *data,int* anal_cnt);
  char charFromDataf(unsigned char *data,int* anal_cnt);
  char reverseFromDataf(unsigned char *data,int* anal_cnt,int r_num);
  int intFromDataf(unsigned char *data,int* anal_cnt);
  
};


#endif