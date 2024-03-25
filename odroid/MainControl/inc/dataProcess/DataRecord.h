#ifndef DATARECORD_H
#define DATARECORD_H
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "UART/uart_communicate.h"
#include <stdio.h>
#include <iostream>

class DataRecord{
public:
    DataRecord(LowlevelCmd *cmd, LowlevelState *state, VecInt2 *_contact, Vec2 *_phase);
    ~DataRecord();
    void getData(float Vcmd, Vec3 pcd, Vec32 posFeet2BGoal,Vec32 forceFeetBody);
    void writeData();

    void readData();
private:
    FILE *fp;
    long long startTime;
    pthread_t _tid;
    LowlevelCmd *_cmd;
    LowlevelState *_state;
    Vec2 *_phase;
    VecInt2 *_contact;

    float _Vcmd;
    Vec3 _pcd;
    Vec32 _posFeet2BGoal,_forceFeetBody;


    static void* runDataRecord(void *arg);
    void* run(void *arg);

};



#endif 