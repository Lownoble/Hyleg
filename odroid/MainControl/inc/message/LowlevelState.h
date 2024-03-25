#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "common/enumClass.h"
#include "interface/CmdPanel.h"

struct MotorState
{
    int ID;
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

struct Treadmile{
    float speed;            //跑步机速度
    float distance;         //超声测得距离
    float distanceGoal;     //理想位置
    float Kp;
    float Kd;
    float Ki;

    Treadmile(){
        speed = 0;
        distance = 0;
        distanceGoal = 0;
        Kp = 0;
        Kd = 0;
        Ki = 0.0;
    }
};

struct LowlevelState
{
    MotorState motorState[4];
    UserCommand userCmd;
    UserValue userValue;
    VecInt2 footContact;
    Treadmile treadmile;
    float pressure;
    float current;

    //获取关节角度
    Mat2 getQ(){
        Mat2 qLegs;
        for(int i(0); i<2; ++i){
            qLegs.col(i)(0) = motorState[i*2+0].q;
            qLegs.col(i)(1) = motorState[i*2+1].q;
        }
        return qLegs;
    }

    //获取关节角速度
    Mat2 getQd(){
        Mat2 qdLegs;
        for(int i(0); i<2; ++i){
            qdLegs.col(i)(0) = motorState[i*2+0].dq;
            qdLegs.col(i)(1) = motorState[i*2+1].dq;           
        }
        return qdLegs;
    }

    RotMat getRotMat(){
        RotMat R = I3;
        return R;
    }

    RotMat getYRotMat(float qy){
        RotMat R;
        R <<    cos(qy), 0, sin(qy),
                0, 1, 0,
                -sin(qy), 0, cos(qy);
        return R;
    }

    void setQ(Vec4 q){
        for(int i(0); i<4; ++i){
            motorState[i].q = q(i);
        }
    }

    float getTreadmileSpeed(){
        return treadmile.speed;
    }

    float getTreadmileDistance(){
        return treadmile.distance;
    }

    void setTreadmileDistanceGoal(float distanceGoal){
        treadmile.distanceGoal = distanceGoal;
    }

    void setTreadmileDistanceGoal(){
        treadmile.distanceGoal = treadmile.distance;
    }

    void setTreadmileKPID(float Kp, float Ki, float Kd){
        treadmile.Kp = saturation(Kp,Vec2(0,1.2));
        treadmile.Ki = saturation(Ki,Vec2(0,1.2));
        treadmile.Kd = saturation(Kd,Vec2(0,1.2));
    }

};



#endif  //LOWLEVELSTATE_HPP