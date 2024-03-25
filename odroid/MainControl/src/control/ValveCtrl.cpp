#include "control/ValveCtrl.h"

ValveCtrl::ValveCtrl(VecInt2 *contact, Vec2 *phase, double stancePhaseRatio)
        :_contact(contact), _phase(phase), _stancePhaseRatio(stancePhaseRatio){
            pressureGoal = 20.0;
            K = 0;
            startPosition = 0.46;
}

ValveCtrl::~ValveCtrl(){
}

void ValveCtrl::calcWaveFrequency(){

}

VecInt2 ValveCtrl::runValveCtrl(WaveStatus status, double pressure, Vec3 Positon){
    calcBodyPhase();
    calcValveSignal(status, pressure);
    if(valveSignal != valveSignal_Last)    startPosition = 0.454;
    calcBodyPosition();
    valveSignal_Last = valveSignal;
    return valveSignal;
}

void ValveCtrl::calcValveSignal(WaveStatus status, double pressure){
    deltaPhase = _stancePhaseRatio-0.5;
    deltaPhase = saturation(deltaPhase, Vec2((_stancePhaseRatio-0.5)/2,_stancePhaseRatio-0.5));
    if(status == WaveStatus::WAVE_ALL){
        for(int i=0; i<2; i++){
            // if((*_contact)(i) == 1 && (*_phase)(i)>deltaPhase && (*_phase)(i)<(1-deltaPhase))   valveSignal(i) = 1;
            if((*_contact)(i) == 1 && (*_contact)(!i) == 0)   valveSignal(i) = 1;
            else    valveSignal(i) = 0;
        }
    }
    else{
        valveSignal.setZero();
    }
    // printf("%d %d %f %f %d %d \n",(*_contact)(0),(*_contact)(1),(*_phase)(0),(*_phase)(1),valveSignal(0),valveSignal(1));
}

void ValveCtrl::calcBodyPhase(){
    Vec2 phase;
    for(int i=0; i<2; i++){
        if((*_contact)(i) == 1)     phase(i) = _stancePhaseRatio * (*_phase)(i);
        else                        phase(i) = (1-_stancePhaseRatio) * (*_phase)(i);
    }
    if((*_contact)(0) == 1 && (*_contact)(1) == 1)  Phase = (phase(0)+phase(1)+1-_stancePhaseRatio);
    else                                            Phase = (phase(0)+phase(1));
    // printf("%f\n",Phase-floor(Phase));
    Phase = Phase-floor(Phase);
}


void ValveCtrl::calcBodyPosition(){
    if(valveSignal(0) ==0 && valveSignal(1) == 0){
        if(0<Phase && Phase<deltaPhase)      zPosition = (0.46 - startPosition)/2 * sin(Phase*M_PI/2/deltaPhase) + (0.46 + startPosition)/2;
        if(1-deltaPhase<Phase && Phase<1)    zPosition = (0.46 - startPosition)/2 * sin((Phase-1)*M_PI/2/deltaPhase)+ (0.46 + startPosition)/2;
    }
    else    zPosition = 0.46;
    if((*_contact)(0) == 1 && (*_contact)(1) == 1 && (*_phase)(0) ==0.5 &&(*_phase)(1) ==0.5)   zPosition = 0.46;

}