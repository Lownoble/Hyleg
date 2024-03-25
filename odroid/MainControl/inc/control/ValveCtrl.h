#ifndef VALVECTRL_H
#define VALVECTRL_H

#include "common/mathTypes.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

#define LOADMASS 0
#define DESIREPRESSURE 0

class ValveCtrl{
public:
    ValveCtrl(VecInt2 *contact, Vec2 *phase, double stancePhaseRatio);
    ~ValveCtrl();
    void calcWaveFrequency();
    VecInt2 runValveCtrl(WaveStatus status, double pressure, Vec3 Positon);
    double getBodyPhase(){return Phase;};
    double getBodyPosition(){return zPosition;};
    
private:
    double _stancePhaseRatio = 0.6;
    double K,pressureGoal = 0;
    Vec2 *_phase;
    VecInt2 *_contact;

    double Phase,deltaPhase;
    double zPosition,startPosition;
    VecInt2 valveSignal, valveSignal_Last;

    void calcValveSignal(WaveStatus status, double pressure);
    void calcBodyPhase();
    void calcBodyPosition();
};


#endif