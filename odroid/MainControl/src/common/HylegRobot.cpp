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

/**
 * [计算关节角度]
 * @param    vecP                   [足端位置]
 * @return   q                      [关节角度]
 */
Vec4 BipedalRobot::getQ(const Vec32 &vecP){
    Vec4 q;
    for(int i(0); i < 2; ++i){
        q.segment(2*i, 2) = _Legs[i]->calcQ(vecP.col(i));
    }
    return q;
}

/**
 * [计算关节角速度]
 * @param    pos                    [足端位置]
 * @param    vel                    [足端速度]
 * @return   qd                     [关节角速度]
 */
Vec4 BipedalRobot::getQd(const Vec32 &pos, const Vec32 &vel){
    Vec4 qd;
    Mat2 q;
    for(int i(0); i < 2; ++i){
        q.col(i) = _Legs[i]->calcQ(pos.col(i));
        qd.segment(2*i, 2) = _Legs[i]->calcQd(q.col(i), vel.col(i));
    }
    return qd;
}

/**
 * [计算关节角速度]
 * @param    q                      [关节角度]
 * @param    feetForce              [足端力]
 * @return   tau                    [关节扭矩]
 */
Vec4 BipedalRobot::getTau(const Vec4 &q, const Vec32 feetForce){
    Vec4 tau;
    for(int i(0); i < 2; ++i){
        tau.segment(2*i, 2) = _Legs[i]->calcTau(q.segment(2*i, 2), feetForce.col(i));
    }
    return tau;
}

// 计算足端位置
Vec3 BipedalRobot::getFootPosition(LowlevelState &state, int id){
    Mat2 qLegs= state.getQ();

    return _Legs[id]->calcPEe2H(qLegs.col(id));

}

// 计算足端速度
Vec3 BipedalRobot::getFootVelocity(LowlevelState &state, int id){
    Mat2 qLegs = state.getQ();
    Mat2 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// 计算足端相对机身的位置
Vec32 BipedalRobot::getFeet2BPositions(LowlevelState &state){
    Vec32 feetPos;
    for(int i(0); i<2; ++i){
        feetPos.col(i) = getFootPosition(state, i);
    }
    return feetPos;
}

// 计算足端相对机身的速度
Vec32 BipedalRobot::getFeet2BVelocities(LowlevelState &state){
    Vec32 feetVel;
    for(int i(0); i<2; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }
    return feetVel;
}

// 计算雅可比矩阵
Mat2 BipedalRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

// 构建HyRobot机器人
HyRobot::HyRobot(){
    _Legs[0] = new HyLeg(0, Vec3( 0, 0, 0));
    _Legs[1] = new HyLeg(1, Vec3( 0, 0, 0));

    _feetPosNormalStand <<  0.0000,  0.0000,
                            0.0000,  0.0000,
                           -0.4500, -0.4500;


    _robVelLimitX << -0.5, 0.5;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;
    _robVelLimitTau << -54, 54;

#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 16;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.4625, 0.4040, 0.5065).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 13.4;
    _pcb << 0.0, 0.0, 0.0;
    _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}