#include "FSM/State_Walking.h"
#include <iomanip>

State_Walking::State_Walking(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::WALKING, "walking"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.080;

#ifdef ROBOT_TYPE_Hy
    _Kpp = Vec3(70, 0, 70).asDiagonal();
    _Kdp = Vec3(10, 0, 0).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(80, 0, 80).asDiagonal();
    _KdSwing = Vec3(0, 0, 0).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

State_Walking::~State_Walking(){
    delete _gait;
}

void State_Walking::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _vCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
}

void State_Walking::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Walking::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::WALKING;
    }
}

void State_Walking::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    _userValue = _lowState->userValue;
    getUserCmd();
    getTreadmileVel();
    calcCmd();
    printf("%f ",_userValue.lx);

    _gait->setGait(_vCmdGlobal(0), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    // printf("%f %f ",_posBody(0),_posBody(2));
    // printf("%f %f ",_velBody(0),_velBody(2));
    // printf("%f %f ",_posFeetGlobalGoal(0,0),_posFeetGlobalGoal(2,0));
    // printf("%f %f ",_velFeetGlobalGoal(0,0),_velFeetGlobalGoal(2,0));
    // printf("%f %f ",_posFeetGlobalGoal(0,1),_posFeetGlobalGoal(2,1));
    // printf("%f %f %f %f ",_qGoal(0),_qGoal(1),_qGoal(2),_qGoal(3));
    // printf("%f %f %f %f ",_qdGoal(0),_qdGoal(1),_qdGoal(2),_qdGoal(3));

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_qGoal);
    _lowCmd->setQd(_qdGoal);

    for(int i(0); i<2; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
    printf("\n");
}    

bool State_Walking::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||1){
        return true;
    }else{
        return false;
    }
}

void State_Walking::setHighCmd(double vx){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0; 
}

void State_Walking::getUserCmd(){
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
}

void State_Walking::getTreadmileVel(){
    _vCmdBody(0) = invNormalize(_userValue.lx + treadmile.speed + treadmile.Kp*(treadmile.distance-treadmile.distance0), _vxLim(0), _vxLim(1));
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
    printf("%f %f %f ",treadmile.Kp,_vCmdBody(0),_userValue.lx + treadmile.speed + treadmile.Kp*(treadmile.distance-treadmile.distance0));
}

void State_Walking::calcCmd(){
    /* Movement */
    _vCmdGlobal =  _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;
}
void State_Walking::calcQQd(){
    Vec32 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
    _qdGoal = _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal);
    // printf("%f %f %f %f ",_posFeetGlobalGoal(0,0),_posFeetGlobalGoal(2,0),_posFeetGlobalGoal(1,0),_posFeetGlobalGoal(1,2));
    // printf("%f %f ",_posBody(0),_posBody(2));
    // printf("%f %f %f %f \n",_posFeet2BGoal(0,0),_posFeet2BGoal(2,0),_posFeet2BGoal(0,1),_posFeet2BGoal(2,1));
    // printf("%f %f %f %f ",_velFeet2BGoal(0,0),_velFeet2BGoal(2,0),_velFeet2BGoal(0,1),_velFeet2BGoal(2,1));
}

void State_Walking::calcTau(){

    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;
    // printf("pos: %f %f %f %f\n",_pcd(0),_pcd(2),_posBody(0),_posBody(2));
    // printf("vel: %f %f %f %f\n",_vCmdGlobal(0),_vCmdGlobal(2),_velBody(0),_velBody(2));
    // printf("Error: %f %f %f %f\n",_posError(0),_posError(2),_velError(0),_velError(2));
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd.setZero();
    //_dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _dWbd.setZero();

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i=0;i<2;i++){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = - _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    // for(int i=0;i<2;i++){
    //     _forceFeetGlobal.col(i) = - _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
    // }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

    // for(int i=0;i<4;i++){
    //     _tau(i) = 0;
    // }
    printf("%d %d ",(*_contact)(0),(*_contact)(1));
    printf("%f %f %f %f ",_forceFeetBody(0,0),_forceFeetBody(2,0),_forceFeetBody(0,1),_forceFeetBody(2,1));
    printf("%f %f %f %f",_tau(0),_tau(1),_tau(2),_tau(3));
}