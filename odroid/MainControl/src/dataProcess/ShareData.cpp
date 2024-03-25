#include "dataProcess/ShareData.h"


ShareData::ShareData() {
        pthread_mutex_init(&mutex_, nullptr);
        pthread_cond_init(&conditionVariable_, nullptr);
}

ShareData::~ShareData() {
        pthread_mutex_destroy(&mutex_);
        pthread_cond_destroy(&conditionVariable_);
    }

void ShareData::setCmd(const LowlevelCmd *cmd){
    pthread_mutex_lock(&mutex_);
    for(int i=0; i<4; i++){
        _lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q    = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq   = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].Kp   = cmd->motorCmd[i].Kp;
        _lowCmd.motorCmd[i].Kd   = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].tau  = cmd->motorCmd[i].tau;
    }
    _lowCmd.valveSignal = cmd->valveSignal;
    dataReady_cmd = true;
    pthread_cond_signal(&conditionVariable_);
    pthread_mutex_unlock(&mutex_);
}

void ShareData::getState(LowlevelState *state){
    pthread_mutex_lock(&mutex_);
    // if(!dataReady_state)    return;
    dataReady_state = false;
    for(int i=0; i<4; i++){
        state->motorState[i].mode = _lowState.motorState[i].mode;
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    state->footContact = _lowState.footContact;
    state->treadmile.speed = _lowState.treadmile.speed;
    state->treadmile.distance = _lowState.treadmile.distance;
    state->pressure = _lowState.pressure;
    state->current = _lowState.current;
    pthread_mutex_unlock(&mutex_);
}

void ShareData::setState(SPI *spi){
    pthread_mutex_lock(&mutex_);
    spi->GetRecv(&_lowState);
    dataReady_state = true;
    pthread_cond_signal(&conditionVariable_);
    pthread_mutex_unlock(&mutex_);
}

void ShareData::getCmd(SPI *spi){
    pthread_mutex_lock(&mutex_);
    // if(!dataReady_cmd)    return;
    dataReady_cmd = false;
    spi->SetSend(&_lowCmd);
    pthread_mutex_unlock(&mutex_);
}

void ShareData::setSensor(Sensor *sensor){
    pthread_mutex_lock(&mutex_);
    // _lowState.treadmile.speed = sensor->treadmile.speed;
    _lowState.treadmile.speed = saturation(sensor->treadmile.speed, Vec2(_lowState.treadmile.speed-0.003, _lowState.treadmile.speed+0.003));
    // if(fabs(sensor->treadmile.speed-_lowState.treadmile.speed)<0.05)
    _lowState.treadmile.speed = last_speed*0.8 + _lowState.treadmile.speed*0.2;
    last_speed = _lowState.treadmile.speed;
    // _lowState.treadmile.distance = sensor->treadmile.distance;
    _lowState.treadmile.distance = saturation(sensor->treadmile.distance, Vec2(_lowState.treadmile.distance-0.005, _lowState.treadmile.distance+0.005));
    _lowState.pressure = sensor->pressure;
    _lowState.current = sensor->current;
    pthread_cond_signal(&conditionVariable_);
    pthread_mutex_unlock(&mutex_);
}

void ShareData::getValveCtrl(Valve *valve){
    pthread_mutex_lock(&mutex_);
    // if(!dataReady_cmd)    return;
    dataReady_cmd = false;
    valve->SetSend(&_lowCmd);
    pthread_mutex_unlock(&mutex_);
}