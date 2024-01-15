#include "common/HyLeg.h"
#include <iostream>

BipedalLeg::BipedalLeg(int legID, float hipLinkLength, 
                           float kneeLinkLength, Vec3 pHip2B)
            :
             _hipLinkLength(hipLinkLength), 
             _kneeLinkLength(kneeLinkLength), 
             _pHip2B(pHip2B){
    if (legID == 0 || legID == 2)
        _sideSign = 1;
    else if (legID == 1 || legID == 3)
        _sideSign = -1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }
}

// Forward Kinematics
Vec3 BipedalLeg::calcPEe2H(Vec2 q){
    float l1 = _hipLinkLength;
    float l2 = _kneeLinkLength;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));

    float c12 = c1*c2 - s1*s2;
    float s12 = s1*c2 + c1*s2;

    Vec3 pEe2H;
    pEe2H(0) = -l1*s1 + l2*c12;
    pEe2H(1) = 0;
    pEe2H(2) = -l1*c1 - l2*s12;

    return pEe2H;
}

Vec3 BipedalLeg::calcPEe2B(Vec2 q){
    return _pHip2B + calcPEe2H(q);
}

// Inverse Kinematics
Vec2 BipedalLeg::calcQ(Vec3 pEe){
    float l = _hipLinkLength;
    Vec3 pEe2H = pEe;
    float q1, q2;
    Vec2 qResult;
    float x, y, z;

    x = pEe2H(0);
    y = pEe2H(1);
    z = pEe2H(2);
 	q1 = 2*atan((2*pow(l,2) - 2*l*x - (2*pow(l,2)*(sqrt(-(pow(x,2) + pow(z,2))*(- 4*pow(l,2) + pow(x,2) + pow(z,2))) - 2*pow(l,2)))/(- 2*pow(l,2) + pow(x,2) + pow(z,2)) + (pow(x,2)*(sqrt(-(pow(x,2) + pow(z,2))*(- 4*pow(l,2) + pow(x,2) + pow(z,2))) - 2*pow(l,2)))/(- 2*pow(l,2) + pow(x,2) + pow(z,2)) + (pow(z,2)*(sqrt(-(pow(x,2) + pow(z,2))*(- 4*pow(l,2) + pow(x,2) + pow(z,2))) - 2*pow(l,2)))/(- 2*pow(l,2) + pow(x,2) + pow(z,2)))/(pow(x,2) + pow(z,2) - 2*l*z));
    q2 = -2*atan((sqrt(-(pow(x,2) + pow(z,2))*(- 4*pow(l,2) + pow(x,2) + pow(z,2))) - 2*pow(l,2))/(- 2*pow(l,2) + pow(x,2) + pow(z,2)));
    
    qResult(0) = q1;
    qResult(1) = q2;

    return qResult;
}

// Jacobian Matrix
Mat2 BipedalLeg::calcJaco(Vec2 q){
    Mat2 jaco;

    float l1 = _hipLinkLength;
    float l2 = _kneeLinkLength;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));

    jaco(0, 0) = -l1*c1 - l2*s1*c2 - l2*c1*s2;
    jaco(1, 0) = l1*s1 - l2*c1*c2 + l2*s1*s2;
    jaco(0, 1) = -l2*c1*s2 - l2*s1*c2;
    jaco(1, 1) = l2*s1*s2 - l2*c1*c2;

    return jaco;
}

// Derivative Forward Kinematics
Vec3 BipedalLeg::calcVEe(Vec2 q, Vec2 qd){
    Vec2 vEe_xz = calcJaco(q) * qd;
    Vec3 vEe;
    vEe(0) = vEe_xz(0);
    vEe(1) = 0;
    vEe(2) = vEe_xz(1);
    return vEe;
}

// Derivative Inverse Kinematics
Vec2 BipedalLeg::calcQd(Vec2 q, Vec3 vEe){
    Vec2 vEe_xz;
    vEe_xz(0) = vEe(0);
    vEe_xz(1) = vEe(2);
    return calcJaco(q).inverse() * vEe_xz;
}

// Inverse Dynamics
Vec2 BipedalLeg::calcTau(Vec2 q, Vec3 force){
    Vec2 force_xz;
    force_xz(0) = force(0);
    force_xz(1) = force(2);
    return calcJaco(q).transpose() * force_xz;
}
