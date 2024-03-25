#include "FSM/State_Horizonal.h"

State_Horizonal::State_Horizonal(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::HORIZONAL, "horizonal"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact),_phase(ctrlComp->phase){

    _xMax = 0.3;
    _xMin = -_xMax;
    _massMax = 10;
    _massMin = 0;

    _Kpp = Vec3(200, 0, 300).asDiagonal();
    _Kdp = Vec3(0, 0, 25).asDiagonal();

}

void State_Horizonal::enter(){
    _pcdInit << _ctrlComp->Mass_x, 0, _ctrlComp->H;
    _pcd = _pcdInit;
    _posFeetGlobalGoal.setZero();
    _velFeetGlobalGoal.setZero();
    _XD=0; _ZD=0;
    _massLoad = 0;

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_Horizonal::run(){
    _posFeetGlobal = _robModel->getFeet2BPositions(*_lowState);
    _posBody << -(_posFeetGlobal(0,0)+_posFeetGlobal(0,1))/2, 0, -(_posFeetGlobal(2,0)+_posFeetGlobal(2,1))/2;
    _velBody.setZero();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    _userValue = _lowState->userValue;

    

    // _XD = invNormalize(_userValue.lx, _xMin, _xMax);     //W S
    _XD = invNormalize(_userValue.ry, _xMin, _xMax);        //I K
    _massLoad = invNormalize(_userValue.ml, _massMin, _massMax, 0, 1);

    printf("%f ",_userValue.ml);

    runGait();

    calcQQd();
    calcTau();

    _ctrlComp->setAllStable();

    _lowCmd->setStableGain();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_qGoal);
    printf("%f %d %f %f %f %f %f %f %f\n",_massLoad,(*_contact)(0),(*_phase)(0),_pcd(2),_posBody(2),_forceFeetBody(0,0),_forceFeetBody(0,1),_forceFeetBody(2,0),_forceFeetBody(2,1));
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
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
}

void State_Horizonal::calcTau(){
    contact(0)=1; contact(1)=1;
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd.setZero();

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, contact, _massLoad);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Horizonal::runGait(){
    _pcd(0) = _XD*(sin(2*M_PI*(*_phase)(0))) + _pcdInit(0);
}

