#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "common/HylegRobot.h"
//#include "common/LowPassFilter.h"
#include "Gait/WaveGenerator.h"
#include "message/LowlevelState.h"
#include "string"

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

class Estimator{
public:
    Estimator(BipedalRobot *robotModel, LowlevelState* lowState, VecInt2 *contact, Vec2 *phase, double dt);
    Estimator(BipedalRobot *robotModel, LowlevelState* lowState, VecInt2 *contact, Vec2 *phase, double dt, Vec18 Qdig, std::string testName);
    ~Estimator();
    Vec3  getPosition();
    Vec3  getVelocity();
    Vec3  getFootPos(int i);
    Vec32 getFeetPos();
    Vec32 getFeetVel();
    Vec32 getPosFeet2BGlobal();
    void run();
    Vec3 getRealPosition();

#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG
private:
    void _initSystem();
    int origin;
    Vec3 position;
    Vec3 velocity;

    Vec32 _feetPosGlobalKine, _feetVelGlobalKine;
    LowlevelState* _lowState;
    BipedalRobot *_robModel;
    Vec2 *_phase;
    VecInt2 *_contact;
    double _dt;
    Vec18 _Qdig;  
    std::string _estName;
    Vec3 _startP;
};

#endif  // ESTIMATOR_H