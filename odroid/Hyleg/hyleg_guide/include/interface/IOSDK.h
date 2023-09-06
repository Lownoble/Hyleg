#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include "hyleg_sdk/hyleg_sdk.h"

class IOSDK : public IOInterface{
public:
    IOSDK();
    ~IOSDK(){};
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
private:
    HYLEG_SDK::UDP  _udp;
    HYLEG_SDK::Safety   _safe;
    HYLEG_SDK::LowCmd   _lowcmd;
    HYLEG_SDK::LowState _lowstate;
};



#endif  // IOSDK_H