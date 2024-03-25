#include "FSM/State_Walking.h"
#include <iomanip>

State_Walking::State_Walking(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::WALKING, "walking"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
              _balCtrl(ctrlComp->balCtrl),_valveCtrl(ctrlComp->valveCtrl){
    _gait = new GaitGenerator(ctrlComp);
    _gaitHeight = 0.060;

    dataRecord = new DataRecord(_lowCmd,_lowState,_contact,_phase);
    errorCtrl = new ErrorCtrl(_contact,_phase);

#ifdef ROBOT_TYPE_Hy
    _Kpp = Vec3(200, 0, 50).asDiagonal();
    _Kdp = Vec3(0, 0, 0).asDiagonal();
    _kpw = 0;
    _Kdw = Vec3(0, 0, 0).asDiagonal();
    _KpSwing = Vec3(50, 0, 100).asDiagonal();
    _KdSwing = Vec3(2, 0, 2).asDiagonal();
    _KpStable = Vec3(1000, 0, 0).asDiagonal();
    _KdStable = Vec3(0, 0, 0).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
    _tauLim = _robModel->getRobTauLimit();
}

State_Walking::~State_Walking(){
    delete _gait;
    delete dataRecord;
    delete errorCtrl;
}

void State_Walking::enter(){
    printf("--------------walking--------------\n");
    _pcd.setZero();
    _pcd(0) = _ctrlComp->Mass_x;
    _pcd(2) = _ctrlComp->H;
    _vCmdBody.setZero();
    _vCmdGlobal.setZero();
    _posFeetGlobalGoal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
    _lowState->setTreadmileKPID(0,0,1);
    _lowState->setTreadmileDistanceGoal();
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
    //获取机器人状态信息
    _posBody = _est->getPosition();                     //机身位置
    _velBody = _est->getVelocity();                     //机身速度
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();      //足端相对机身位置
    _posFeetGlobal = _est->getFeetPos();                //足端位置
    _velFeetGlobal = _est->getFeetVel();                //足端速度
    _B2G_RotMat = _lowState->getRotMat();               //机身旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();

    //获取目标速度
    _userValue = _lowState->userValue;
    getUserCmd();
    treadmile = _lowState->treadmile;
    getTreadmileVel();
    _bodyPhase = _valveCtrl->getBodyPhase();
    calcCmd();
    calcPcd();

    //计算行走步态（足端位置、足端速度）
    _gait->setGait(_vCmdGlobal(0) , _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    //计算关节力矩、角度、角速度
    calcTau();
    calcQQd();

    // _qCompensate = errorCtrl->calcErrorCompensate(_qGoal,_q);
    dataRecord->getData(_vCmdGlobal(0),_pcd,_posFeet2BGoal,_forceFeetBody);


    printf("%d %d ",(*_contact)(0),(*_contact)(1));
    // printf("%f %f %f ",(*_phase)(0),(*_phase)(1),_bodyPhase);
    printf("pcd:%f %f ",_pcd(0),_pcd(2));
    printf("posbody:%f %f ",_posBody(0),_posBody(2));
    printf("velbody:%f vCmd:%f ",_velBody(0),_vCmdGlobal(0));
    // printf("%f %f ",_posFeetGlobalGoal(0,0),_posFeetGlobalGoal(2,0));
    // printf("%f %f ",_velFeetGlobalGoal(0,0),_velFeetGlobalGoal(2,0));
    // printf("%f %f ",_posFeetGlobalGoal(0,1),_posFeetGlobalGoal(2,1));
    // printf("%f %f %f %f ",_qGoal(0),_qGoal(1),_qGoal(2),_qGoal(3));
    // printf("%f %f %f %f ",_q(0),_q(1),_q(2),_q(3));
    // printf("%f %f %f %f ",_qCompensate(0),_qCompensate(1),_qCompensate(2),_qCompensate(3));
    printf("forceFeetBody:%.1f %.1f %.1f %.1f ",_forceFeetBody(0,0),_forceFeetBody(2,0),_forceFeetBody(0,1),_forceFeetBody(2,1));
    // printf("tau:%f %f %f %f ",_tau(0),_tau(1),_tau(2),_tau(3));
    printf("speed:%f pressure:%f distance:%f loadMass:%f",treadmile.speed,_lowState->pressure,treadmile.distance,loadMass);
    // printf("valveSignal:%d %d ",_lowCmd->valveSignal(0),_lowCmd->valveSignal(1));

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_qGoal);
    // _lowCmd->setQ(_qGoal + _qCompensate);
    _lowCmd->setQd(_qdGoal);

    for(int i(0); i<2; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

    //双足支撑相设置KP
    // if((*_contact)(0) == 1 && (*_contact)(1) == 1){
    //     // _lowCmd->setQ(_qGoal + _qCompensate);
    //     _lowCmd->setDoubleStableGain(0);
    //     _lowCmd->setDoubleStableGain(1);
    // }

    //设置阀出力时期KP
    // for(int i(0); i<2; i++){
    //     if(_lowCmd->valveSignal(i) == 1)     _lowCmd->setValveCtrlGain(i);
    // }

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

// 设置目标速度
void State_Walking::setHighCmd(double vx){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0; 
}

// 获取PC输入速度
void State_Walking::getUserCmd(){
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
}

// 获取跑步机速度
void State_Walking::getTreadmileVel(){
    _vCmdBody(0) = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1)) + treadmile.Kd * treadmile.speed - saturation(treadmile.Ki * (treadmile.distanceGoal - treadmile.distance), Vec2(-0.05,0.05));
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
}

// 计算速度命令
void State_Walking::calcCmd(){
    /* Movement */
    _vCmdGlobal =  _vCmdBody;
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    
}

// 计算机身目标位置
void State_Walking::calcPcd(){
    /* Movement */
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.10, _posBody(0) + 0.10));
    // _pcd(2) = _valveCtrl->getBodyPosition();
}


// 计算关节角度、角速度
void State_Walking::calcQQd(){
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _vCmdGlobal);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
    _qdGoal = _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal);
}

// 计算关节扭矩
void State_Walking::calcTau(){

    _posError = _pcd - _posBody;            //位置误差
    // printf("_posError:%f %f ",_posError(0),_posError(2));
    _velError = _vCmdGlobal - _velBody;     //速度误差
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    //_dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());
    _dWbd.setZero();

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    //液压阀出力计算
    // if(_lowCmd->valveSignal(0) && _lowCmd->valveSignal(1))
    //     valveForce = 2 *_lowState->pressure * 10000 * 0.016*0.016*M_PI*0.48;
    // else if((_lowCmd->valveSignal(0) && !_lowCmd->valveSignal(1)) || (!_lowCmd->valveSignal(0) && _lowCmd->valveSignal(1)))
    //     valveForce = _lowState->pressure * 10000 * 0.016*0.016*M_PI*0.48;
    // else
    valveForce = saturation(valveForce, Vec2(0,150));
    valveForce = 0;
    //负重 kg
    loadMass = invNormalize(_userValue.ml, 0, 40, 0, 2);
    // loadMass = 0;

    //支撑腿足端力计算
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact, loadMass-valveForce/9.8);

    // printf("valveForce:%f pressure:%f",valveForce,  _lowState->pressure);


    // _forceFeetGlobal.setZero();
    for(int i=0;i<2;i++){
        if((*_contact)(i) == 0){//摆动腿修正力
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
        else if((*_contact)(i) == 1){
            _forceFeetGlobal(0,i) = 0;
            // _forceFeetGlobal.col(i) += _KpStable*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdStable*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
        if((*_contact)(i) == 1 && (*_contact)(!i) == 0){//单腿支撑腿偏置
            // _forceFeetGlobal(0,i) -= 10;
            _forceFeetGlobal(2,i) = 10;
        }
        if((*_contact)(i) == 1 && (*_contact)(!i) == 1){//双腿支撑腿
            // _forceFeetGlobal(0,i) -= 10/2;
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

    for(int i=0;i<4;i++){
        _tau(i) = saturation(_tau(i), _tauLim);
        // _tau(i) = 0; //no tau output
    }

}