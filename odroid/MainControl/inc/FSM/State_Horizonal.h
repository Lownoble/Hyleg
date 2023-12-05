#ifndef HORIZONAL_H
#define HORIZONAL_H

#include "FSM/FSMState.h"

class State_Horizonal : public FSMState{
public:
    State_Horizonal(CtrlComponents *ctrlComp);
    ~State_Horizonal(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    void calcQQd();
    void calcTau();
    void runGait();

    Estimator *_est;
    BipedalRobot *_robModel;
    BalanceCtrl *_balCtrl;

    VecInt2 *_contact;
    Vec2 *_phase;

    Vec3 _pcd, _pcdInit;
    Vec3 _ddPcd, _dWbd;
    Mat3 _Kpp, _Kdp;

    Vec4 _q, _tau;
    Vec3 _posBody, _velBody;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec32 _posFeetGlobal, _velFeetGlobal;
    Vec32 _posFeet2BGlobal;
    
    Vec4 _qGoal, _qdGoal; 
    Vec32 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec32 _posFeet2BGoal, _velFeet2BGoal;
    Vec32 _forceFeetGlobal, _forceFeetBody;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
};

#endif  // HORIZONAL_H