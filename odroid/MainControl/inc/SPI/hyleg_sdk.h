#ifndef _HYLEG_SDK_H_
#define _HYLEG_SDK_H_

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
//#include "comm.h"
// #include "safety.h"
// #include "udp.h"
// #include "loop.h"
// #include "quadruped.h"
// #include "joystick.h"
// #include <boost/bind.hpp>

#define HL HYLEG_SDK  // short name

namespace HYLEG_SDK
{
    typedef struct
    {
        LowlevelState lowState;
    }LowState;

    typedef struct
    {
        LowlevelCmd lowCmd;
    }LowCmd;

}


#endif