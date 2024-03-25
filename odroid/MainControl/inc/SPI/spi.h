#ifndef SPI_H
#define SPI_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sstream>
#include <math.h>
#include "SPI/Tmotor.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "UART/uart_communicate.h"

#define SPI_BUF_SIZE 256 


class SPI{
public:
    SPI();
    ~SPI();
    void transfer(int sel);

    void SetSend(const LowlevelCmd *cmd){
        for(int i=0;i<4;i++){
            motorCmd[i+1].ID = i+1;
            motorCmd[i+1].mode = cmd->motorCmd[i].mode;
            motorCmd[i+1].q    = cmd->motorCmd[i].q;
            motorCmd[i+1].dq   = cmd->motorCmd[i].dq;
            motorCmd[i+1].Kp   = cmd->motorCmd[i].Kp;
            motorCmd[i+1].Kd   = cmd->motorCmd[i].Kd;
            motorCmd[i+1].tau  = cmd->motorCmd[i].tau;
        }
    }

    void GetRecv(LowlevelState *state){
        for(int i=0; i<4; i++){
            state->motorState[i].mode = motorState[i+1].mode;
            state->motorState[i].q = motorState[i+1].q;
            state->motorState[i].dq = motorState[i+1].dq;
            state->motorState[i].ddq = motorState[i+1].ddq;
            state->motorState[i].tauEst = motorState[i+1].tauEst;
        }
        state->footContact = footContact;
        current = state->current;
        pressure = state->pressure;
        velocity = state->treadmile.speed;
    }

private:
    MotorCmd motorCmd[5];
    MotorState motorState[5];
    VecInt2 footContact;
    float current;
    float pressure;
    float velocity;
    #define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

    //SPI setting
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    // uint32_t speed = 20000000;
    uint16_t delay = 0;
    uint8_t cs = 0;

    //spi buf
    uint8_t spi_tx_buf[SPI_BUF_SIZE] = {0};
    uint8_t spi_rx_buf[SPI_BUF_SIZE] = {0};
    int spi_tx_cnt = 0;
    int spi_rx_cnt = 0;
    int _fd;

    int spi_init();

    void setDataInt_spi(int i);
    void setDataFloat_spi(float f);
    float floatFromData_spi(unsigned char *data, int *anal_cnt);
    char charFromData_spi(unsigned char *data, int *anal_cnt);
    int intFromData_spi(unsigned char *data, int *anal_cnt);
    void can_board_send(int sel);
    void slave_rx(uint8_t *data_buf, int num);

};

#endif


