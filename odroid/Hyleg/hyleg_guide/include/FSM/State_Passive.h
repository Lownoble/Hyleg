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
}