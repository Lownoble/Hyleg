#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

GaitGenerator::~GaitGenerator(){}

void GaitGenerator::setGait(float vGoalGlobal,  float gaitHeight){
    _vGoal = vGoalGlobal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vGoal = 0;
}

void GaitGenerator::run(Vec32 &feetPos, Vec32 &feetVel){
    // if(_firstRun){
    //     _startP = _est->getFeetPos();
    //     _firstRun = false;
    //     #ifdef COMPILE_DEBUG
    //     _startT = getSystemTime();
    //     std::vector<std::string> labels = {"Curve 1", "Curve 2"};
    //     _testGaitPlot.addPlot("MyPlot", 2, labels);
    //     #endif
    // }
    for(int i(0); i<2; ++i){
        if((*_phase)(i) < 0.01){
            deltaX = _vGoal*_waveG->getTstance()/2;
            if((*_contact)(i) == 1) _startP.col(i)(0) = (_est->getPositionGoal())(0) + deltaX;
            if((*_contact)(i) == 0) _startP.col(i)(0) = (_est->getPositionGoal())(0) - deltaX;
            _startP.col(i)(2) = 0;
            // printf("SP:%f ",_startP.col(i)(0));
        }
        if((*_contact)(i) == 1){
            // if((*_phase)(i) < 0.5){
            //     _startP.col(i) = _est->getFootPos(i);
            //     _startP.col(i)(2) = 0;
            // }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vGoal, (*_phase)(i));
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
        if(_contactPast(i)==1 && (*_contact)(i) == 0)   changeOrigin = true;
    }
    #ifdef COMPILE_DEBUG
    _passT = (double)(getSystemTime() - _startT) * 1e-6;
    _testGaitPlot.addFrame("MyPlot", _passT, (*_contact)(1));
    _testGaitPlot.addFrame("MyPlot", _passT, (*_contact)(2));
    _testGaitPlot.showPlot("MyPlot");
    #endif
    _pastP = feetPos;
    _phasePast = *_phase;
    _contactPast = *_contact;
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = 0;
    footPos(2) = cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = 0;
    footVel(2) = cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

float GaitGenerator::cycloidXPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}