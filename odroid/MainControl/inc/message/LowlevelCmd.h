#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorCmd{
    int ID;
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[4];

    void setQ(Vec4 q){
        for(int i(0); i<4; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec2 qi){
        motorCmd[legID*2+0].q = qi(0);
        motorCmd[legID*2+1].q = qi(2);
    }

    void setQd(Vec4 qd){
        for(int i(0); i<4; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec2 qdi){
        motorCmd[legID*2+0].dq = qdi(0);
        motorCmd[legID*2+1].dq = qdi(1);
    }

    void setTau(Vec4 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<4; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }

    void setZeroDq(int legID){
        motorCmd[legID*2+0].dq = 0;
        motorCmd[legID*2+1].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<2; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*2+0].tau = 0;
        motorCmd[legID*2+1].tau = 0;
    }
    void setZeroTau(){
        for(int i(0); i<2; ++i){
            setZeroTau(i);
        }
    }

    void setSimStanceGain(int legID){
        motorCmd[legID*2+0].mode = 10;
        motorCmd[legID*2+0].Kp = 180;
        motorCmd[legID*2+0].Kd = 8;
        motorCmd[legID*2+1].mode = 10;
        motorCmd[legID*2+1].Kp = 180;
        motorCmd[legID*2+1].Kd = 8;
    }

    void setRealStanceGain(int legID){
        motorCmd[legID*2+0].mode = 10;
        //motorCmd[legID*2+0].Kp = 30;
        motorCmd[legID*2+0].Kd = 2;
        motorCmd[legID*2+1].mode = 10;
        //motorCmd[legID*2+1].Kp = 30;
        motorCmd[legID*2+1].Kd = 2;
    }

    void setZeroGain(int legID){
        motorCmd[legID*2+0].mode = 10;
        motorCmd[legID*2+0].Kp = 0;
        motorCmd[legID*2+0].Kd = 0;
        motorCmd[legID*2+1].mode = 10;
        motorCmd[legID*2+1].Kp = 0;
        motorCmd[legID*2+1].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<2; ++i){
            setZeroGain(i);
        }
    }

    void setStableGain(int legID){
        motorCmd[legID*2+0].mode = 10;
        motorCmd[legID*2+0].Kp = 50;
        motorCmd[legID*2+0].Kd = 0.8;
        motorCmd[legID*2+1].mode = 10;
        motorCmd[legID*2+1].Kp = 50;
        motorCmd[legID*2+1].Kd = 0.8;
    }

    void setStableGain(){
        for(int i(0); i<2; ++i){
            setStableGain(i);
        }
    }

    void setSwingGain(int legID){
        motorCmd[legID*2+0].mode = 10;
        motorCmd[legID*2+0].Kp = 200;
        motorCmd[legID*2+0].Kd = 0.8;
        motorCmd[legID*2+1].mode = 10;
        motorCmd[legID*2+1].Kp = 200;
        motorCmd[legID*2+1].Kd = 0.8;
    }
};




#endif  //LOWLEVELCMD_H