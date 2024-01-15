#ifndef FEETENDCAL_H
#define FEETENDCAL_H

#include "control/CtrlComponents.h"
#include "message/LowlevelState.h"

class FeetEndCal{
public:
    FeetEndCal(CtrlComponents *ctrlComp);
    ~FeetEndCal();
    Vec3 calFootPos(int legID, float vGoalGlobal,  float phase);
private:
    LowlevelState *_lowState;
    Estimator *_est;
    BipedalRobot *_robModel;

    Vec3 _nextStep, _footPos;
    Vec3 _bodyVelGlobal;        // linear velocity
    Vec3 _bodyAccGlobal;        // linear accelerator
    Vec3 _bodyWGlobal;          // angular velocity
    Vec3 _bodyPosGlobal;        // actual position
    Vec3 xGoalGlobal;           // desire position

    Vec4 _feetRadius, _feetInitAngle;
    float _Tstance, _Tswing;
    float _kx, _ky;
};

#endif  // FEETENDCAL_H