#include "common/HylegRobot.h"
#include <iostream>

Vec3 BipedalRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0);
}

Vec32 BipedalRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec32 vecXP;
    Mat2 qLegs = state.getQ();
    for(int i(0); i < 2; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematics
Vec4 BipedalRobot::getQ(const Vec32 &vecP){
    Vec4 q;
    for(int i(0); i < 2; ++i){
        q.segment(2*i, 2) = _Legs[i]->calcQ(vecP.col(i));
    }
    return q;
}

Vec4 BipedalRobot::getQd(const Vec32 &pos, const Vec32 &vel){
    Vec4 qd;
    Mat2 q;
    for(int i(0); i < 2; ++i){
        q.col(i) = _Legs[i]->calcQ(pos.col(i));
        qd.segment(2*i, 2) = _Legs[i]->calcQd(q.col(i), vel.col(i));
    }
    return qd;
}

Vec4 BipedalRobot::getTau(const Vec4 &q, const Vec32 feetForce){
    Vec4 tau;
    for(int i(0); i < 2; ++i){
        tau.segment(2*i, 2) = _Legs[i]->calcTau(q.segment(2*i, 2), feetForce.col(i));
    }
    return tau;
}

// Forward Kinematics
Vec3 BipedalRobot::getFootPosition(LowlevelState &state, int id){
    Mat2 qLegs= state.getQ();

    return _Legs[id]->calcPEe2H(qLegs.col(id));

}

// Forward derivative Kinematics
Vec3 BipedalRobot::getFootVelocity(LowlevelState &state, int id){
    Mat2 qLegs = state.getQ();
    Mat2 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// Forward Kinematics
Vec32 BipedalRobot::getFeet2BPositions(LowlevelState &state){
    Vec32 feetPos;
    for(int i(0); i<2; ++i){
        feetPos.col(i) = getFootPosition(state, i);
    }
    return feetPos;
}

Vec32 BipedalRobot::getFeet2BVelocities(LowlevelState &state){
    Vec32 feetVel;
    for(int i(0); i<2; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }
    return feetVel;
}

Mat2 BipedalRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

HyRobot::HyRobot(){
    _Legs[0] = new HyLeg(0, Vec3( 0, 0, 0));
    _Legs[1] = new HyLeg(1, Vec3( 0, 0, 0));

    _feetPosNormalStand <<  0.0000,  0.0000,
                            0.0000,  0.0000,
                           -0.5400, -0.5400;


    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;

#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 12.5;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 13.4;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}