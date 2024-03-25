#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"
#include "common/HyLeg.h"


class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void getTargetPos();
    Vec4 _targetQ;
    Vec32 _targetPos,_posFeetGlobalGoal,_posFeet2BGoal;
    Vec3 _pcd;
    RotMat _B2G_RotMat, _G2B_RotMat;
    float _startPos[4];
    float _duration = 1000;
    float _percent = 0;
    float _targetKp  = 200;
    float _startKp[4];
};

#endif  // FIXEDSTAND_H