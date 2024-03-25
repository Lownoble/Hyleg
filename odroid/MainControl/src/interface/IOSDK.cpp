#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include "UART/uart_communicate.h"
#include <stdio.h>

#ifdef ROBOT_TYPE_Hyleg
IOSDK::IOSDK():_safe(HYLEG_SDK::LeggedType::Hyleg), _udp(HYLEG_SDK::LOWLEVEL){
    std::cout << "The control interface for real robot" << std::endl;
    _udp.InitCmdData(_lowCmd);

}
#endif
IOSDK::IOSDK():IOInterface(){
    std::cout << "The control interface for real robot: Hyleg\n" << std::endl;
    cmdPanel = new KeyBoard();
}

IOSDK::~IOSDK(){
    delete cmdPanel;
}


void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    // sendCmd(cmd);
    // recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
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
    _lowCmd.valveSignal = cmd->valveSignal;
    // spi->SetSend(&_lowCmd);
    // valve.SetSend(&_lowCmd);
}

void IOSDK::recvState(LowlevelState *state){
    // spi->GetRecv(&_lowState);
    // sensor.GetRecv(&_lowState);
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
    state->footContact = _lowState.footContact;
    state->treadmile.speed = _lowState.treadmile.speed;
    state->treadmile.distance = _lowState.treadmile.distance;
    state->pressure = _lowState.pressure;
    state->current = _lowState.current;
    // printf("speed:%f distance:%f\n",_lowState.treadmile.speed,_lowState.treadmile.distance);
}
