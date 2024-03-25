#include "dataProcess/DataRecord.h"
#include "common/timeMarker.h"

DataRecord::DataRecord(LowlevelCmd *cmd, LowlevelState *state, VecInt2 *contact, Vec2 *phase)
    :_cmd(cmd), _state(state), _contact(contact), _phase(phase){
    fp = fopen("../../data/record.txt", "w");
    if (fp == NULL) {
        printf("文件打开失败！\n");
    }
    startTime = getSystemTime();

    pthread_create(&_tid, NULL, runDataRecord, (void*)this);
}

DataRecord::~DataRecord(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    fclose(fp);
}

void DataRecord::getData(float Vcmd, Vec3 pcd, Vec32 posFeet2BGoal,Vec32 forceFeetBody){
    _Vcmd = Vcmd;
    _pcd = pcd;
    _posFeet2BGoal = posFeet2BGoal;
    _forceFeetBody = forceFeetBody;
}

void DataRecord::writeData(){
    fprintf(fp, "%f\t", (getSystemTime()-startTime)/1000000.0);
    fprintf(fp, "%d\t%d\t%.4f\t%.4f\t",(*_contact)(0),(*_contact)(1),(*_phase)(0),(*_phase)(1));
    fprintf(fp, "%.4f\t%.4f\t%.4f\t",_Vcmd, _pcd(0), _pcd(2));
    fprintf(fp, "%f\t%f\t%f\t%f\t",_posFeet2BGoal(0,0),_posFeet2BGoal(2,0),_posFeet2BGoal(0,1),_posFeet2BGoal(2,1));
    fprintf(fp, "%.2f\t%.2f\t%.2f\t%.2f\t",_forceFeetBody(0,0),_forceFeetBody(2,0),_forceFeetBody(0,1),_forceFeetBody(2,1));
    for(int i=0; i<4; i++){
        fprintf(fp, "%.4f\t%.4f\t",_state->motorState[i].q,_state->motorState[i].tauEst);
    }
    // for(int i=0; i<4; i++){
    //     fprintf(fp, "%d\t%f\t%f\t%f\t",cmd->motorCmd[i].ID,cmd->motorCmd[i].q,cmd->motorCmd[i].dq,cmd->motorCmd[i].tau);
    // }
    fprintf(fp, "%.4f\t%.4f\t%.4f\t",_state->treadmile.speed,_state->pressure,_state->current);
    fprintf(fp,"\n");
}

void DataRecord::readData(){

}


void* DataRecord::runDataRecord(void *arg){
    ((DataRecord*)arg)->run(NULL);
    return NULL;
}

void* DataRecord::run(void *arg){
    while(1){
        writeData();
        usleep(10000);
    }
    return NULL;
}
