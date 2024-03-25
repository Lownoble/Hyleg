#include "FSM/State_SwingTest.h"

State_SwingTest::State_SwingTest(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::SWINGTEST, "swingTest"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.080;

#ifdef ROBOT_TYPE_Hy
    _KpSwing = Vec3(500, 0, 200).asDiagonal();
    _KdSwing = Vec3(3, 0, 3).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
}

State_SwingTest::~State_SwingTest(){
    delete _gait;
}

void State_SwingTest::enter(){
    _pcd.setZero();
    _pcd(0) = _ctrlComp->Mass_x;
    _pcd(2) = _ctrlComp->H;
    _vCmdBody.setZero();
    _vCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _startT = getSystemTime();
    _period = 4.0;
    firstRun = 1;
}

void State_SwingTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_SwingTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::SWINGTEST;
    }
}

void State_SwingTest::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    _passT = (double)(getSystemTime() - _startT) * 1e-6;
    phase = fmod(_passT + _period, _period) / _period;

    _userValue = _lowState->userValue;
    getUserCmd();
    calcCmd();

    // _gait->setGait(_vCmdGlobal(0), _gaitHeight);
    // _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    // gaitCircle(0.05);
    // gaitSquare(0.08,0.08);
    gaitPoint(0,0);


    calcTau();
    calcQQd();


    // printf("%d %d ",(*_contact)(0),(*_contact)(1));
    // printf("%f  ",phase);
    printf("pcd:%f %f ",_pcd(0),_pcd(2));
    printf("posbody:%f %f ",_posBody(0),_posBody(2));
    printf("footPosition:%f %f ",_posFeet2BGoal(0,1),_posFeet2BGoal(2,1));
    // printf("%f %f ",_velFeetGlobalGoal(0,0),_velFeetGlobalGoal(2,0));
    // printf("%f %f ",_velFeetGlobal(0,0),_velFeetGlobal(2,0));
    // printf("%f %f ",_posFeetGlobalGoal(0,1),_posFeetGlobalGoal(2,1));
    // printf("%f %f %f %f ",_qdGoal(0),_qdGoal(1),_qdGoal(2),_qdGoal(3));
    printf("forceFeetBody:%.1f %.1f %.1f %.1f ",_forceFeetBody(0,0),_forceFeetBody(2,0),_forceFeetBody(0,1),_forceFeetBody(2,1));
    printf("tau:%f %f %f %f ",_tau(0),_tau(1),_tau(2),_tau(3));

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_qGoal);
    _lowCmd->setQd(_qdGoal);

    for(int i(0); i<2; ++i){
        _lowCmd->setSwingGain(i);
    }
    printf("\n");
}    

bool State_SwingTest::checkStepOrNot(){
    if(1){
        return true;
    }else{
        return false;
    }
}

void State_SwingTest::setHighCmd(double vx){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0; 
}

void State_SwingTest::getUserCmd(){
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = 0;
    _vCmdBody(2) = 0;
}

void State_SwingTest::calcCmd(){
    /* Movement */
    _vCmdGlobal =  _vCmdBody;
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _pcd(0) = _pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt;
    
}
void State_SwingTest::calcQQd(){
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState);

    for(int i=0; i<2; i++){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _vCmdGlobal);
    }
    _qGoal = _robModel->getQ(_posFeet2BGoal);
    _qdGoal = _robModel->getQd(_posFeet2BGoal, _velFeet2BGoal);
}

void State_SwingTest::calcTau(){

    for(int i=0;i<2;i++){
        _forceFeetGlobal.col(i) =  _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = Mat2ToVec4(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

}


void State_SwingTest::gaitCircle(float radius){
    float phasePI = 2 * M_PI * phase;
    if(firstRun){
        for(int i=0; i<2; i++){
            _posFeetGlobalGoal(0,i) = radius * sin(phasePI/4);
            _posFeetGlobalGoal(2,i) = 0;
            _velFeetGlobalGoal(0,i) = radius * cos(phasePI/4);
            _velFeetGlobalGoal(2,i) = 0;
        }
        if(phase>0.95) firstRun = 0;
    }
    else{
        for(int i=0; i<2; i++){
            _posFeetGlobalGoal(0,i) = radius * sin(phasePI) + radius;
            _posFeetGlobalGoal(2,i) = radius - radius*cos(phasePI);
            _velFeetGlobalGoal(0,i) = radius * cos(phasePI);
            _velFeetGlobalGoal(2,i) = radius * sin(phasePI);
        }
    }

}

void State_SwingTest::gaitSquare(float length, float width){
    float phasePI = 4 * M_PI * (fmod(phase, 0.25)-0.125);
    for(int i=0; i<2; i++){
        if(phase<=0.25){
            _posFeetGlobalGoal(0,i) = (length * sin(phasePI) + length)/2;
            _posFeetGlobalGoal(2,i) = 0;
            _velFeetGlobalGoal(0,i) = length * cos(phasePI)/2;
            _velFeetGlobalGoal(2,i) = 0;
        }
        else if(phase>0.25 && phase<=0.50){
            _posFeetGlobalGoal(0,i) = length;
            _posFeetGlobalGoal(2,i) = (width * sin(phasePI) +width)/2;
            _velFeetGlobalGoal(0,i) = 0;
            _velFeetGlobalGoal(2,i) = width * cos(phasePI)/2;
        }
        else if(phase>0.50 && phase<=0.75){
            _posFeetGlobalGoal(0,i) = (length - length * sin(phasePI))/2;
            _posFeetGlobalGoal(2,i) = width;
            _velFeetGlobalGoal(0,i) = -length * cos(phasePI)/2;
            _velFeetGlobalGoal(2,i) = 0;
        }
        else if(phase>0.75 && phase<=1.00){
            _posFeetGlobalGoal(0,i) = 0;
            _posFeetGlobalGoal(2,i) = (width - width * sin(phasePI))/2;
            _velFeetGlobalGoal(0,i) = 0;
            _velFeetGlobalGoal(2,i) = - width * cos(phasePI)/2;
        }
    }

}

void State_SwingTest::gaitPoint(float pointX, float pointZ){
    for(int i=0; i<2; i++){
        _posFeetGlobalGoal(0,i) = pointX;
        _posFeetGlobalGoal(2,i) = pointZ;
        _velFeetGlobalGoal(0,i) = 0;
        _velFeetGlobalGoal(2,i) = 0;
    }
}