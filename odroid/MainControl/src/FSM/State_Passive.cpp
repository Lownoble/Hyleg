#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
    :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    printf("----------------passive----------------\n");
    if(_ctrlComp -> ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<4; i++){
            _lowCmd -> motorCmd[i].mode = 10;
            _lowCmd -> motorCmd[i].q = 0;
            _lowCmd -> motorCmd[i].dq = 0;
            _lowCmd -> motorCmd[i].Kp = 0;
            _lowCmd -> motorCmd[i].Kd = 8;
            _lowCmd -> motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp -> ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<4; i++){
            _startKp[i] = _lowCmd -> motorCmd[i].Kp;
            _lowCmd -> motorCmd[i].q = _lowState -> motorState[i].q;
            _lowCmd -> motorCmd[i].dq = 0;
            _lowCmd -> motorCmd[i].Kp = _lowCmd -> motorCmd[i].Kp;
            _lowCmd -> motorCmd[i].Kd = 5;
            _lowCmd -> motorCmd[i].tau = 0;
        }
    }
    _ctrlComp -> setAllSwing();
}

void State_Passive::run(){
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1:_percent;
    // for(int j=0; j<4; j++){
    //     _lowCmd->motorCmd[j].Kp = (1-_percent)*_startKp[j];
    //     // _lowCmd->motorCmd[j].q = _lowState -> motorState[j].q;
    // }
    _lowCmd->motorCmd[1].Kp = (1-_percent)*_startKp[1];
    _lowCmd->motorCmd[3].Kp = (1-_percent)*_startKp[3];

    if(_percent==1){
        for(int i=0; i<4; i++){
            _lowCmd -> motorCmd[i].mode = 10;
            _lowCmd -> motorCmd[i].q = 0;
            _lowCmd -> motorCmd[i].dq = 0;
            _lowCmd -> motorCmd[i].Kp = 0;
            _lowCmd -> motorCmd[i].Kd = 5;
            _lowCmd -> motorCmd[i].tau = 0;
        }
    }
}

void State_Passive::exit(){
    _percent = 0;
}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}