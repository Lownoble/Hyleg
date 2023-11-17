#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "common/enumClass.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec2 bias);
    ~WaveGenerator();
    void calcContactPhase(Vec2 &phaseResult, VecInt2 &contactResult, WaveStatus status);
    float getTstance();
    float getTswing();
    float getT();
private:
    void calcWave(Vec2 &phase, VecInt2 &contact, WaveStatus status);

    double _period;
    double _stRatio;
    Vec2 _bias;

    Vec2 _normalT;                   // [0, 1)
    Vec2 _phase, _phasePast;
    VecInt2 _contact, _contactPast;
    VecInt2 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast;

    double _passT;                   // unit: second
    long long _startT;    // unit: us
#ifdef COMPILE_DEBUG
    PyPlot _testPlot;
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H