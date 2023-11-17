#ifndef HYLEGROBOT_H
#define HYLEGROBOT_H

#include "common/HyLeg.h"
#include "message/LowlevelState.h"

class BipedalRobot{
public:
    BipedalRobot(){};
    ~BipedalRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec32 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec4 getQ(const Vec32 &feetPosition);
    Vec4 getQd(const Vec32 &feetPosition, const Vec32 &feetVelocity);
    Vec4 getTau(const Vec4 &q, const Vec32 feetForce);

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id);
    Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec32 getFeet2BPositions(LowlevelState &state);
    Vec32 getFeet2BVelocities(LowlevelState &state);

    Mat2 getJaco(LowlevelState &state, int legID);
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec32 getFeetPosIdeal(){return _feetPosNormalStand;}
    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    BipedalLeg* _Legs[2];
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
    Vec32 _feetPosNormalStand;
    double _mass;
    Vec3 _pcb;
    Mat3 _Ib;
};

class HyRobot : public BipedalRobot{
public:
    HyRobot();
    ~HyRobot(){};
};

#endif