#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct LowlevelState
{
    MotorState motorState[4];

    Vec23 getQ(){
        Vec23 qLegs;
        for(int i(0); i<2; ++i){
            qLegs.col(i)(0) = motorState[i*2+0].q;
            qLegs.col(i)(1) = motorState[i*2+1].q;
        }
        return qLegs;
    }

    Vec23 getQd(){
        Vec23 qdLegs;
        for(int i(0); i<2; ++i){
            qdLegs.col(i)(0) = motorState[i*2+0].qd;
            qdLegs.col(i)(1) = motorState[i*2+1].qd;           
        }
        return qdLegs;
    }

    void setQ(Vec4 q){
        for(int i(0); i<4; ++i){
            motorState[i].q = q(i);
        }
    }
}



#endif  //LOWLEVELSTATE_HPP