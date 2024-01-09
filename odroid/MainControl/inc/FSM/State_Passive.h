#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    float _duration = 1000;
    float _percent = 0;
    float _startKp[4];
};

#endif //PASSIVE_H