#include "control/ErrorCtrl.h"

ErrorCtrl::ErrorCtrl(VecInt2 *contact, Vec2 *phase)
    :_contact(contact), _phase(phase){

        memset(motorErrorSum,0.0,sizeof(motorErrorSum));
        Ki=0.06;
        count = 1;
        _qCompensate.setZero();
}

ErrorCtrl::~ErrorCtrl(){
}

Vec4 ErrorCtrl::calcErrorCompensate(Vec4 _qGoal, Vec4 _q){

    if(count && (*_contact)(0)==1 &&(*_contact)(1)==1){
        return _qCompensate;
    }
    count = 0;
    countL = (int)((*_phase)(0) * 100.0 + (*_contact)(0) * 100);
    countR = (int)((*_phase)(1) * 100.0 + (*_contact)(1) * 100);



    filter_count = 9;
    if(countLPsat!=countL){
        _qCompensate(0) = motorErrorSum[0][countL];
        _qCompensate(1) = motorErrorSum[1][countL];
        for(int i=1; i<filter_count; i++){
            _qCompensate(0) += motorErrorSum[0][(countL+i)%200];
            _qCompensate(1) += motorErrorSum[1][(countL+i)%200];
        }
        _qCompensate(0) = _qCompensate(0) *Ki /filter_count;
        _qCompensate(1) = _qCompensate(1) *Ki /filter_count;
    }
    if(countRPsat!=countR){
        _qCompensate(2) = motorErrorSum[2][countR];
        _qCompensate(3) = motorErrorSum[3][countR];
        for(int i=1; i<filter_count; i++){
            _qCompensate(2) += motorErrorSum[2][(countR+i)%200];
            _qCompensate(3) += motorErrorSum[3][(countR+i)%200];
        }
        _qCompensate(2) = _qCompensate(2) *Ki /filter_count;
        _qCompensate(3) = _qCompensate(3) *Ki /filter_count;
    }

    filter_count = 4;
    if(countLPsat!=countL){
        for(int i=1; i<filter_count; i++){
            _qCompensate(0) += _qCompensate_last[0][(countL-i+200)%200];
            _qCompensate(1) += _qCompensate_last[1][(countL-i+200)%200];
        }
        _qCompensate(0) = _qCompensate(0) /filter_count;
        _qCompensate(1) = _qCompensate(1) /filter_count;
    }
    if(countRPsat!=countR){
        for(int i=1; i<filter_count; i++){
            _qCompensate(2) += _qCompensate_last[2][(countR-i+200)%200];
            _qCompensate(3) += _qCompensate_last[3][(countR-i+200)%200];
        }
        _qCompensate(2) = _qCompensate(2) /filter_count;
        _qCompensate(3) = _qCompensate(3) /filter_count;
    }

    motorErrorSum[0][countL] += _qGoal(0) - _q(0);
    motorErrorSum[1][countL] += _qGoal(1) - _q(1);
    motorErrorSum[2][countR] += _qGoal(2) - _q(2);
    motorErrorSum[3][countR] += _qGoal(3) - _q(3);

    if((*_contact)(0) == 0){
        motorErrorSum[0][countL] = 0;
        motorErrorSum[1][countL] = 0;        
    }
    if((*_contact)(1) == 0){
        motorErrorSum[2][countR] = 0;
        motorErrorSum[3][countR] = 0;        
    }

    countLPsat = countL;
    countRPsat = countR;

    _qCompensate_last[0][countL] = _qCompensate(0);
    _qCompensate_last[1][countL] = _qCompensate(1);
    _qCompensate_last[2][countR] = _qCompensate(2);
    _qCompensate_last[3][countR] = _qCompensate(3);

    for(int i=0; i<4; i++){
        _qCompensate(i) = saturation(_qCompensate(i),Vec2(-0.1,0.1));
    }
    // if(fabs(_qCompensate_last(2)-_qCompensate(2))>0.02){
    //     printf("_qCompensate:%f %f ",_qCompensate(2),_qCompensate(3));
    //     printf("%f %f %f %f ",_qGoal(0),_qGoal(1),_qGoal(2),_qGoal(3));
    //     printf("%f %f %f %f ",_q(0),_q(1),_q(2),_q(3));
    //     printf("%f %f\n",motorErrorSum[2][countR],motorErrorSum[3][countR]);
    // }
    
    // _qCompensate_last = _qCompensate;
    return _qCompensate;
}
