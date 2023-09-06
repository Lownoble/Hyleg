#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include <stdio.h>

#ifdef ROBOT_TYPE_Hyleg
IOSDK::IOSDK():_safe(HYLEG_SDK::LeggedType::Hyleg), _udp(HYLEG_SDK::LOWLEVEL){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);

}
#endif

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    for(int i=0; i<4; i++){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;        
    }

    _udp.SetSend(_lowCmd);
    _udp.Send();

    _udp.Recv();
    _udp.GetRecv(_lowState);

    for(int i=0; i<4; i++){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->motorState[i].mode = _lowState.motorState[i].mode;        
    }

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}