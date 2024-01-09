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
#include "SPI/spi_node.h"
#include "SPI/Tmotor.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

static void pabort(const char *s);
int spi_init();
void transfer(int fd, int sel);

struct SPI{
    MotorCmd motorCmd[5];
    MotorState motorState[5];
    void SPIInit(){
    for(int i=0;i<5;i++){
        motorCmd[i].ID = i;
        motorState[i].ID = i;
    }
    }

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
    }

};
extern SPI _spi;
#endif


