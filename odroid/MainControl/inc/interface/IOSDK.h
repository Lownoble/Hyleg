#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include "SPI/hyleg_sdk.h"

class IOSDK : public IOInterface{
public:
    IOSDK();
    ~IOSDK();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
private:
    void sendCmd(const LowlevelCmd *lowCmd);
    void recvState(LowlevelState *state);

    LowlevelCmd   _lowCmd;
    LowlevelState _lowState;
};



#endif  // IOSDK_H