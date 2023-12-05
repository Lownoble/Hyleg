#include "FSM/State_Horizonal.h"

State_Horizonal::State_Horizonal(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::HORIZONAL, "horizonal"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact),_phase(ctrlComp->phase){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;

    _Kpp = Vec3(150, 150, 150).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

}

void State_Horizonal::enter(){
    _pcdInit = _est->getRealPosition();
    _pcdInit << 0, 0, _ctrlComp->H;
    _pcd = _pcdInit;

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_Horizonal::run(){
    _posBody = _est->getRealPosition();
    _velBody.setZero();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    for(int i(0); i < 2; ++i){
        _posFeetGlobal.col(i) = _posBody + _robModel->getFootPosition(*_lowState, i);
        _velFeetGlobal.col(i) = _velBody + _robModel->getFootVelocity(*_lowState, i);
    }
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    _userValue = _lowState->userValue;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.lx, _xMin, _xMax);
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);

    _posFeetGlobalGoal.setZero();
    _velFeetGlobalGoal.setZero();
    for(int i(0); i < 2; ++i){
        _posFeetGlobalGoal.col(i) << _pcd(0), 0, 0;
    }

    runGait();

    calcQQd();
    calcTau();

     _ctrlComp->setStartWave();

    _lowCmd->setStableGain();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_q);
    printf("%d %f %f %f %f\n",(*_contact)(0),(*_phase)(0),_posFeetGlobalGoal(0,0),_posFeetGlobalGoal(1,0),_posFeetGlobalGoal(2,0));
}

void State_Horizonal::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}


FSMStateName State_Horizonal::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::HORIZONAL;
    }
}

void State_Horizonal::calcQQd(){
    Vec32 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
    _qdGoal = _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal);
}

void State_Horizonal::calcTau(){

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd.setZero();

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Horizonal::runGait(){
    for(int i(0); i < 2; ++i){
        if((*_contact)(0)==1)
            _posFeetGlobalGoal(0,i) = 2*_pcd(0)*(fabs((*_phase)(0)-0.5)-0.5);
        else
            _posFeetGlobalGoal(0,i) = - 2*_pcd(0)*(fabs((*_phase)(0)-0.5)-0.5);
    }
}