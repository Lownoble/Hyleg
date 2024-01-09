#ifndef HYLEG_H
#define HYLEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class BipedalLeg{
public:
    BipedalLeg(int legID, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~BipedalLeg(){}
    Vec3 calcPEe2H(Vec2 q);
    Vec3 calcPEe2B(Vec2 q);
    Vec2 calcQ(Vec3 pEe);
    Vec3 calcVEe(Vec2 q, Vec2 qd);
    Vec2 calcQd(Vec2 q, Vec3 vEe);
    Vec2 calcTau(Vec2 q, Vec3 force);
    Mat2 calcJaco(Vec2 q);
    Vec3 getHip2B(){return _pHip2B;}
protected:
    float _sideSign;
    const float _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};


class HyLeg : public BipedalLeg{
public:
    HyLeg(const int legID, const Vec3 pHip2B):
        BipedalLeg(legID, 0.250, 0.250, pHip2B){}
    ~HyLeg(){}
};

#endif