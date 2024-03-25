#include <iostream>
#include "common/mathTools.h"

class ErrorCtrl{
public:
    ErrorCtrl(VecInt2 *contact, Vec2 *phase);
    ~ErrorCtrl();
    Vec4 calcErrorCompensate(Vec4 _qGoal, Vec4 _q);

private:
    double motorErrorSum[4][200] = {{0.0}};
    double motorError_last[4][200] = {{0.0}};
    double _qCompensate_last[4][200] = {{0.0}};
    double Kp=0, Ki=0, Kd=0;             
    int count,countL,countR,countLPsat,countRPsat;
    int filter_count;

    Vec2 *_phase;
    VecInt2 *_contact;

    Vec4 _qCompensate;
    Vec4 motorError;


};