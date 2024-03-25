#ifndef SHAREDATA
#define SHAREDATA

#include <iostream>
#include <pthread.h>
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "SPI/spi.h"
#include "UART/uart_communicate.h"

class ShareData {
public:
    ShareData();
    ~ShareData();

    void setCmd(const LowlevelCmd *cmd);
    void getState(LowlevelState *state);
    void setState(SPI *spi);
    void getCmd(SPI *spi);
    void setSensor(Sensor *sensor);
    void getValveCtrl(Valve *valve);

private:
    int data_;
    bool dataReady_state = false;
    bool dataReady_cmd = false;
    pthread_mutex_t mutex_;
    pthread_cond_t conditionVariable_;
    LowlevelCmd   _lowCmd;
    LowlevelState _lowState;
    Sensor _sensor;
    Valve _valve;
    float last_speed;

};


#endif