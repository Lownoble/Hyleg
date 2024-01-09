#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"
#include "Gait/FeetEndCal.h"

#ifdef COMPILE_DEBUG
#include <common/PyPlot.h>
#endif  // COMPILE_DEBUG

/*cycloid gait*/
class GaitGenerator{
public:
    GaitGenerator(CtrlComponents *ctrlComp);
    ~GaitGenerator();
    void setGait(float vxyGoalGlobal, float gaitHeight);
    void run(Vec32 &feetPos, Vec32 &feetVel);
    Vec3 getFootPos(int i);
    Vec3 getFootVel(int i);
    void restart();
private:
    float cycloidXPosition(float startX, float endX, float phase);
    float cycloidXVelocity(float startX, float endX, float phase);
    float cycloidZPosition(float startZ, float height, float phase);
    float cycloidZVelocity(float height, float phase);

    WaveGenerator *_waveG;
    Estimator *_est;
    FeetEndCal *_feetCal;
    BipedalRobot *_robModel;
    LowlevelState *_state;
    float _gaitHeight;
    float _vGoal;
    Vec2 *_phase, _phasePast;
    VecInt2 *_contact, _contactPast;
    Vec32 _startP, _endP, _idealP, _pastP;
    bool _firstRun;
    bool changeOrigin;
    float deltaX;

    double _passT;                   // unit: second
    long long _startT;    // unit: us
#ifdef COMPILE_DEBUG
    PyPlot _testGaitPlot;
#endif  // COMPILE_DEBUG

};

#endif  // GAITGENERATOR_H