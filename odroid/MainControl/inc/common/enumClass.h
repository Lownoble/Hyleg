#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT
};

enum class RobotType{
    Hyleg,

};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // walking          4
    L2_A,       // fixedStand       2
    L2_B,       // passive          1
    L2_X,       // freeStand        3
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    L1_X,       // HORIZONAL        0
    L1_A,       // swingTest        9
    L1_Y        // stepTest         8
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL,
    STABLE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    //EXIT,
    INVALID,
    PASSIVE,        //0
    FIXEDSTAND,     //1
    FREESTAND,
    WALKING,        //4
    HORIZONAL,      //0
    SWINGTEST,      //9
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,
#endif      //COMPILE_WITH_MOVE_BASE
    
    STEPTEST
};

#endif  // ENUMCLASS_H