#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/enumClass.h"

Estimator::Estimator(BipedalRobot *robotModel, LowlevelState* lowState, 
                     VecInt2 *contact, Vec2 *phase, double dt, Vec18 Qdig,
                     std::string testName)
          :_robModel(robotModel), _lowState(lowState), _contact(contact),
           _phase(phase), _dt(dt), _Qdig(Qdig), _estName(testName){

    _initSystem();
}

Estimator::Estimator(BipedalRobot *robotModel, LowlevelState* lowState, 
                     VecInt2 *contact, Vec2 *phase, double dt)
          :_robModel(robotModel), _lowState(lowState), _contact(contact), 
           _phase(phase), _dt(dt){

    for(int i(0); i<_Qdig.rows(); ++i){
        if(i < 3){
            _Qdig(i) = 0.0003;
        }
        else if(i < 6){
            _Qdig(i) = 0.0003;
        }
        else{
            _Qdig(i) = 0.01;
        }
    }

    _estName = "current";

    _vxLim = _robModel->getRobVelLimitX();

    _initSystem();

}

Estimator::~Estimator(){}

void Estimator::_initSystem(){
    origin = 0;
    position.setZero();
    velocity.setZero();
    _startP.setZero();
    _pcd.setZero();
}

void Estimator::run(){
    _feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState);
    _feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState);

    _pcd(0) = _pcd(0) + _vCmdBody * _dt;
    _pcd(1) = 0;
    _pcd(2) = 0.54;
    
    for(int i=0;i<2;i++){
        if(_contactPast(i)==1 && (*_contact)(i) == 0){
            origin = !i;
            _startP(0) = _pcd(0);
            _startP(1) = 0;
            _startP(2) = -_feetPosGlobalKine(2,origin);
        }
    }
    _contactPast = *_contact;
    stepLength = _feetPosGlobalKine(0,!origin) - _feetPosGlobalKine(0,origin);
    
    _userValue = _lowState->userValue;
    _vCmdBody = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1));


    position(0) = -_feetPosGlobalKine(0,origin) + _startP(0);
    position(1) = 0;
    position(2) = -_feetPosGlobalKine(2,origin);
    
    velocity(0) = _vCmdBody;
    velocity(1) = 0;
    velocity(2) = -_feetVelGlobalKine(2,origin);

    // printf("contact:%d %d origin:%d phase:%f ",(*_contact)(0),(*_contact)(1),origin,(*_phase)(0));
    // printf("%f %f %f\n",stepLength,_startP(0),position(0));
}

Vec3 Estimator::getPosition(){
    return position;
//    return _xhat.segment(0, 3);
}

Vec3 Estimator::getVelocity(){
    return velocity;
//    return _xhat.segment(3, 3);
}

Vec3 Estimator::getFootPos(int i){
    return getPositionGoal() + _robModel->getFootPosition(*_lowState, i);
}

Vec32 Estimator::getFeetPos(){
    Vec32 feetPos;
    for(int i(0); i < 2; ++i){
        feetPos.col(i) = getFootPos(i);
    }
    return feetPos;
}

Vec32 Estimator::getFeetVel(){
    Vec32 feetVel = _robModel->getFeet2BVelocities(*_lowState);
    for(int i(0); i < 2; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}

Vec32 Estimator::getPosFeet2BGlobal(){
    Vec32 feet2BPos;
    for(int i(0); i < 2; ++i){
        feet2BPos.col(i) = _robModel->getFootPosition(*_lowState, i);
    }
    return feet2BPos;
}

Vec3 Estimator::getPositionGoal(){
    return _pcd;
}