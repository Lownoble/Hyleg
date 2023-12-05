#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
    :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    for(int i=0; i<2; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd -> setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd -> setRealStanceGain(i);
        }
        _lowCmd -> setZeroDq(i);
        _lowCmd -> setZeroTau(i);
    }
    for(int i=0; i<4; i++){
        _lowCmd -> motorCmd[i].q = _lowState -> motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
        _startKp[i] = _lowCmd->motorCmd[i].Kp;
    }
    getTargetPos(0.00,-_ctrlComp->H,0.00,-_ctrlComp->H);
    _ctrlComp -> setAllStance();
}

void State_FixedStand::run(){
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1:_percent;
    for(int j=0; j<4; j++){
        _lowCmd->motorCmd[j].Kp = (1-_percent)*_startKp[j] +_percent*_targetKp;
        _lowCmd->motorCmd[j].q = (1-_percent)*_startPos[j] +_percent*_targetPos[j];
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::WALKING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::HORIZONAL;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
    else{
        return FSMStateName::FIXEDSTAND;
    }
}

void State_FixedStand::getTargetPos(float x1, float z1, float x2, float z2){
   Vec2 Q1 = _Legs[0]->calcQ(Vec3 (x1,0,z1));
   Vec2 Q2 = _Legs[1]->calcQ(Vec3 (x2,0,z2));
   _targetPos[0] = Q1(0); _targetPos[1] = Q1(1);
   _targetPos[2] = Q2(0); _targetPos[3] = Q2(1);
}