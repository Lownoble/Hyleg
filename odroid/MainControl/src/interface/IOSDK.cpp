#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include "SPI/spi.h"
#include <stdio.h>

#ifdef ROBOT_TYPE_Hyleg
IOSDK::IOSDK():_safe(HYLEG_SDK::LeggedType::Hyleg), _udp(HYLEG_SDK::LOWLEVEL){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);

}
#endif
IOSDK::IOSDK():IOInterface(){
    std::cout << "The control interface for real robot: Hyleg" << std::endl;
    cmdPanel = new KeyBoard();
}

IOSDK::~IOSDK(){
    delete cmdPanel;
}


void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    // printf("\n");
}

void IOSDK::sendCmd(const LowlevelCmd *cmd){
    for(int i=0; i<4; i++){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
    }
    _spi.SetSend(&_lowCmd);
}

void IOSDK::recvState(LowlevelState *state){
    _spi.GetRecv(&_lowState);
    // for(int i=0; i<4; i++){
    //     _lowState.motorState[i].q = _lowCmd.motorCmd[i].q;
    //     _lowState.motorState[i].dq = _lowCmd.motorCmd[i].dq;
    // }

    for(int i=0; i<4; i++){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;
    }
}

