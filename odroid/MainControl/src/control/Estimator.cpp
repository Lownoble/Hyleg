#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/enumClass.h"

Estimator::Estimator(BipedalRobot *robotModel, LowlevelState* lowState, WaveGenerator *waveG,
                     VecInt2 *contact, Vec2 *phase, double dt, Vec18 Qdig,
                     std::string testName)
          :_robModel(robotModel), _lowState(lowState),_waveG(waveG), _contact(contact),
           _phase(phase), _dt(dt), _Qdig(Qdig), _estName(testName){

    _initSystem();
}

Estimator::Estimator(BipedalRobot *robotModel, LowlevelState* lowState, WaveGenerator *waveG,
                     VecInt2 *contact, Vec2 *phase, double dt)
          :_robModel(robotModel), _lowState(lowState),_waveG(waveG), _contact(contact), 
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
    _userValue.setZero();
}

void Estimator::run(){
    _feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState);
    _feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState);

    _userValue = _lowState->userValue;
    _vCmdBody = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1));
    #if TREADMILE
    _treadmile = _lowState->treadmile;
    _vCmdBody = invNormalize(_userValue.lx, _vxLim(0), _vxLim(1))+ _treadmile.Kd * _treadmile.speed - saturation(_treadmile.Ki * (_treadmile.distanceGoal - _treadmile.distance), Vec2(-0.05,0.05));; 
    #endif

    _pcd(0) = _pcd(0) + _vCmdBody * _dt;
    _pcd(1) = 0;
    _pcd(2) = 0.45;

    position(0) = position(0) + _vCmdBody * _dt;
    position(1) = 0;
    
    velocity(0) = _vCmdBody;
    velocity(1) = 0;

    #if TREADMILE
    // position(0) = position(0) + _treadmile.Kp * (_treadmile.distanceGoal - _treadmile.distance);
    // velocity(0) = velocity(0) + _treadmile.Ki * (_treadmile.distanceGoal - _treadmile.distance);
    #endif
    

    if((*_contact)(0) == 1 && (*_contact)(1) == 0){//左脚支撑
        origin = 0; 
        position(2) = -_feetPosGlobalKine(2,0);
        velocity(2) = -_feetVelGlobalKine(2,0);
        // _startP.col(0) = position + _robModel->getFootPosition(*_lowState, 0);
        _startP.col(0)(0) = _pcd(0) - _vCmdBody * _waveG->getTstance()*((*_phase)(0)-0.5);
    }
    if((*_contact)(0) == 0 && (*_contact)(1) == 1){//右脚支撑
        origin = 1;
        position(2) = -_feetPosGlobalKine(2,1);
        velocity(2) = -_feetVelGlobalKine(2,1);
        // _startP.col(1) = position + _robModel->getFootPosition(*_lowState, 1);
        _startP.col(1)(0) = _pcd(0) - _vCmdBody * _waveG->getTstance()*((*_phase)(1)-0.5);
    }
    if((*_contact)(0) == 1 && (*_contact)(1) == 1){//双足支撑
        origin = 2;
        position(2) =-(_feetPosGlobalKine(2,0) + _feetPosGlobalKine(2,1))/2;
        velocity(2) =-(_feetVelGlobalKine(2,0) + _feetVelGlobalKine(2,1))/2;
        // _startP.col(0) = position + _robModel->getFootPosition(*_lowState, 0);
        // _startP.col(1) = position + _robModel->getFootPosition(*_lowState, 1);
        _startP.col(0)(0) = _pcd(0) - _vCmdBody * _waveG->getTstance()*((*_phase)(0)-0.5);
        _startP.col(1)(0) = _pcd(0) - _vCmdBody * _waveG->getTstance()*((*_phase)(1)-0.5);
    }

    // printf("contact:%d %d origin:%d phase:%f ",(*_contact)(0),(*_contact)(1),origin,(*_phase)(0));
    // printf("%f %f %f %f\n",_lowState->motorState[0].q,_lowState->motorState[1].q,_lowState->motorState[2].q,_lowState->motorState[3].q);
    // printf("position:%f pcd:%f velocity:%f Vcmd:%f\n",position(0),_pcd(0),velocity(0),_treadmile.speed);
    // printf("%f %f %f\n",_treadmile.speed,_treadmile.distanceGoal,_treadmile.distance);
}

Vec3 Estimator::getPosition(){
    return position;
}

Vec3 Estimator::getVelocity(){
    return velocity;
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

Vec3 Estimator::getPositionGoal(){
    return _pcd;
}

Vec3 Estimator::getStartP(int i){
    return _startP.col(i);
}

void Estimator::restart(){
    position(0) = 0;
    velocity.setZero();
    _startP.setZero();
    _pcd(0) = 0;
}