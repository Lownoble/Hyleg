#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){};
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    float _targetPos[4] = {0.785, 0.785, 0.785, 0.785};
    float _startPos[4];
    float _duration = 1000;
    float _percent = 0;
}

#endif  // FIXEDSTAND_H