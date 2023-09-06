#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
// #include "common/HylegRobot.h"
// #include "Gait/WaveGenerator.h"
// #include "control/Estimator.h"
// #include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0,0,0,0);
        *phase = Vec4(0.5,0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        //...
#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    //...

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    // void runWaveGen(){
    //     waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    // }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    // void geneObj(){

    // }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
}



#endif  // CTRLCOMPONENTS_H