#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator),
             _lowState(ctrlComp->lowState),
             _robModel(ctrlComp->robotModel){
    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.005;

    Vec32 feetPosBody = _robModel->getFeetPosIdeal();
    for(int i(0); i<2; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

FeetEndCal::~FeetEndCal(){}

Vec3 FeetEndCal::calFootPos(int legID, float vGoalGlobal,  float phase){
    _bodyVelGlobal = _est->getVelocity();
    
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vGoalGlobal);
    _nextStep(1) = 0;
    _nextStep(2) = 0;
   // printf("%f ",_nextStep(0));

    _footPos = _est->getPosition() + _nextStep;
    _footPos(2) = 0.0;

    return _footPos;
}