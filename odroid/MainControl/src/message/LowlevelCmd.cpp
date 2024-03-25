#include "message/LowlevelCmd.h"

void LowlevelCmd::setSimStanceGain(int legID){
    motorCmd[legID*2+0].mode = 10;
    motorCmd[legID*2+0].Kp = 180;
    motorCmd[legID*2+0].Kd = 8;
    motorCmd[legID*2+1].mode = 10;
    motorCmd[legID*2+1].Kp = 180;
    motorCmd[legID*2+1].Kd = 8;
}

//设置固定站立模式增益
void LowlevelCmd::setRealStanceGain(int legID){
    motorCmd[legID*2+0].mode = 1;
    //motorCmd[legID*2+0].Kp = 30;
    motorCmd[legID*2+0].Kd = 2;
    motorCmd[legID*2+1].mode = 1;
    //motorCmd[legID*2+1].Kp = 30;
    motorCmd[legID*2+1].Kd = 2;
}

//设置增益为0
void LowlevelCmd::setZeroGain(int legID){
    motorCmd[legID*2+0].mode = 0;
    motorCmd[legID*2+0].Kp = 0;
    motorCmd[legID*2+0].Kd = 0;
    motorCmd[legID*2+1].mode = 0;
    motorCmd[legID*2+1].Kp = 0;
    motorCmd[legID*2+1].Kd = 0;
}
void LowlevelCmd::setZeroGain(){
    for(int i(0); i<2; ++i){
        setZeroGain(i);
    }
}

//设置支撑相增益
void LowlevelCmd::setStableGain(int legID){
    motorCmd[legID*2+0].mode = 3;
    motorCmd[legID*2+0].Kp = 100;
    motorCmd[legID*2+0].Kd = 0.5;
    motorCmd[legID*2+1].mode = 3;
    motorCmd[legID*2+1].Kp = 100;
    motorCmd[legID*2+1].Kd = 0.5;
}

void LowlevelCmd::setStableGain(){
    for(int i(0); i<2; ++i){
        setStableGain(i);
    }
}

//设置摆动相增益
void LowlevelCmd::setSwingGain(int legID){
    motorCmd[legID*2+0].mode = 10;
    motorCmd[legID*2+0].Kp = 200;
    motorCmd[legID*2+0].Kd = 0.8;
    motorCmd[legID*2+1].mode = 10;
    motorCmd[legID*2+1].Kp = 200;
    motorCmd[legID*2+1].Kd = 0.8;
}

//设置双足支撑相增益
void LowlevelCmd::setDoubleStableGain(int legID){
    motorCmd[legID*2+0].mode = 10;
    motorCmd[legID*2+0].Kp = 400;
    motorCmd[legID*2+0].Kd = 1.5;
    motorCmd[legID*2+1].mode = 10;
    motorCmd[legID*2+1].Kp = 400;
    motorCmd[legID*2+1].Kd = 1.5;
}

//设置液压阀出力时的增益
void LowlevelCmd::setValveCtrlGain(int legID){
    motorCmd[legID*2+0].mode = 10;
    motorCmd[legID*2+0].Kp = 0;
    motorCmd[legID*2+0].Kd = 0.8;
    motorCmd[legID*2+1].mode = 10;
    motorCmd[legID*2+1].Kp = 0;
    motorCmd[legID*2+1].Kd = 0.8;
}
