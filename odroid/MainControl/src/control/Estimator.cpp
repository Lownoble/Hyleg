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

    _initSystem();

}

Estimator::~Estimator(){}

void Estimator::_initSystem(){
    origin = 0;
    position.setZero();
    velocity.setZero();
}

void Estimator::run(){
    _feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState);
    _feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState);
    if((*_contact)(0) == 1 && (*_contact)(1) == 0)  origin = 0;
    else if((*_contact)(0) == 0 && (*_contact)(1) == 1) origin = 1;
    else if((*_contact)(0) == 1 && (*_contact)(1) == 1) origin = origin;
    //position = -_feetPosGlobalKine.col(origin); position(1) = 0;
    velocity = -_feetVelGlobalKine.col(origin); velocity(1) = 0;
    if((*_phase)(origin) < 0.02){
        _startP = position;
    }
    UserValue _userValue = _lowState->userValue;
    float _vCmdBody = invNormalize(_userValue.lx, -0.4, 0.4);
    velocity(0) = _vCmdBody;    velocity(2) = 0;
    // position(0) = _startP(0) + _vCmdBody*(*_phase)(origin)*0.6;
    position(0) = position(0) + _vCmdBody * _dt;
    position(2) = 0.54;
    Mat2 qLegs = _lowState->getQ();
    Mat2 qdLegs = _lowState->getQd();
    // printf("%f %f %f %f ",qLegs.col(0)(0),qLegs.col(0)(1),qLegs.col(1)(0),qLegs.col(1)(1));
    // printf("%f %f %f %f ",qdLegs(0,0),qdLegs(1,0),qdLegs(0,1),qdLegs(1,1));
    // printf("%f %f ",position(0),position(2));
    // printf("%f %f \n",velocity(0),velocity(2));
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
    return getPosition() + _robModel->getFootPosition(*_lowState, i);
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