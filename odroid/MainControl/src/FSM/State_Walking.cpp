#include "FSM/State_Walking.h"
#include <iomanip>

State_Walking::State_Walking(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::WALKING, "walking"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
              _balCtrl(ctrlComp->balCtrl),treadmile(ctrlComp->lowState->treadmile){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.080;

#ifdef ROBOT_TYPE_Hy
    _Kpp = Vec3(100, 0, 80).asDiagonal();
    _Kdp = Vec3(0, 0, 0).asDiagonal();
    _kpw = 0; 
    _Kdw = Vec3(0, 0, 0).asDiagonal();
    _KpSwing = Vec3(50, 0, 100).asDiagonal();
    _KdSwing = Vec3(2, 0, 2).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
    _tauLim = _robModel->getRobTauLimit();
}

State_Walking::~State_Walking(){
    delete _gait;
}

void State_Walking::enter(){
    _pcd.setZero();
    _pcd(0) = _ctrlComp->Mass_x;
    _pcd(2) = _ctrlComp->H;
    _vCmdBody.setZero();
    _vCmdGlobal.setZero();
    _posFeetGlobalGoal.setZero();
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

    _gait->setGait(_vCmdGlobal(0), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();


    // printf("%d %d ",(*_contact)(0),(*_contact)(1));
    // printf("%f %f ",_pcd(0),_pcd(2));
    // printf("%f %f ",_posBody(0),_posBody(2));
    // printf("%f %f ",_velBody(0),_vCmdBody(0));
    // printf("%f %f ",_posFeetGlobalGoal(0,0),_posFeetGlobalGoal(2,0));
    // printf("%f %f ",_velFeetGlobalGoal(0,0),_velFeetGlobalGoal(2,0));
    // printf("%f %f ",_posFeetGlobalGoal(0,1),_posFeetGlobalGoal(2,1));
    // printf("%f %f %f %f ",_qdGoal(0),_qdGoal(1),_qdGoal(2),_qdGoal(3));
    // printf("%f %f %f %f ",_forceFeetBody(0,0),_forceFeetBody(2,0),_forceFeetBody(0,1),_forceFeetBody(2,1));
    // printf("%f %f %f %f ",_tau(0),_tau(1),_tau(2),_tau(3));

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
    //printf("\n");
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
    _vCmdBody(0) = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1)) + treadmile.speed;
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
}

void State_Walking::calcCmd(){
    /* Movement */
    _vCmdGlobal =  _vCmdBody;
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt + treadmile.distanceGoal, Vec2(_posBody(0) - 0.10, _posBody(0) + 0.10));
    
}
void State_Walking::calcQQd(){
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _vCmdGlobal);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
    _qdGoal = _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal);
}

void State_Walking::calcTau(){

    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    //_dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());
    _dWbd.setZero();

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i=0;i<2;i++){
        if((*_contact)(i) == 0){//摆动腿修正力
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
        if((*_contact)(i) == 1){//支撑腿偏置
            _forceFeetGlobal(0,i) -= 1;
        }
    }


    // for(int i=0;i<2;i++){
    //     _forceFeetGlobal.col(i) = - _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
    // }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

    for(int i=0;i<4;i++){
        _tau(i) = saturation(_tau(i), _tauLim);
        // _tau(i) = 0; //no tau output
    }

}