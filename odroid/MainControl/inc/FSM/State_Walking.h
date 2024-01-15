#ifndef WALKING_H
#define WALKING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

class State_Walking : public FSMState{
public:
    State_Walking(CtrlComponents *ctrlComp);
    ~State_Walking();
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
    void getTreadmileVel();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    BipedalRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3  _posBody, _velBody;
    Vec32 _posFeetGlobal, _velFeetGlobal;
    Vec32 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec4 _q;
    Vec32 _posFeet2B;
    Treadmile treadmile;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    Vec32 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec32 _posFeet2BGoal, _velFeet2BGoal;
    Vec32 _forceFeetGlobal, _forceFeetBody;
    Vec4 _qGoal, _qdGoal;
    Vec4 _tau;
    Vec3 _ddPcd, _dWbd;
    int origin;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim, _tauLim;
    Vec2 *_phase;
    VecInt2 *_contact;
};

#endif