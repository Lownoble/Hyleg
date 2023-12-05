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
    BipedalLeg* _Legs[2];
    void getTargetPos(float x1, float z1, float x2, float z2);
    float _targetPos[4] = {0.785, 0.0, 0.785, 0.0};
    float _startPos[4];
    float _duration = 1000;
    float _percent = 0;
    float _targetKp  = 200;
    float _startKp[4];
};

#endif  // FIXEDSTAND_H