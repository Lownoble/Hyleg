#ifndef SWINGTEST_H
#define SWINGTEST_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"

class State_SwingTest : public FSMState{
public:
    State_SwingTest(CtrlComponents *ctrlComp);
    ~State_SwingTest();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    bool checkStepOrNot();
    void gaitCircle(float radius);
    void gaitSquare(float length, float width);

    GaitGenerator *_gait;
    Estimator *_est;
    BipedalRobot *_robModel;

    // Rob State
    Vec3  _posBody, _velBody;
    Vec32 _posFeetGlobal, _velFeetGlobal;
    Vec32 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec4 _q;
    Vec32 _posFeet2B;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    Vec32 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec32 _posFeet2BGoal, _velFeet2BGoal;
    Vec32 _forceFeetGlobal, _forceFeetBody;
    Vec4 _qGoal, _qdGoal;
    Vec4 _tau;

    // Control Parameters
    double _gaitHeight;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim;
    Vec2 *_phase;
    VecInt2 *_contact;
    long long _startT;
    double _passT, phase, _period;
    int firstRun;
};

#endif