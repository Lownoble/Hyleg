#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec2 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{

    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5;
    _statusPast = WaveStatus::SWING_ALL;
}

WaveGenerator::~WaveGenerator(){
}

void WaveGenerator::calcContactPhase(Vec2 &phaseResult, VecInt2 &contactResult, WaveStatus status)
{

    calcWave(_phase, _contact, status);
    //printf("contact:%d %d phase:%f %f\n",_contact(0),_contact(1),_phase(0),_phase(1));

    if (status != _statusPast)
    {
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        calcWave(_phasePast, _contactPast, _statusPast);
        //printf("contact:%d %d phase:%f %f\n",_contactPast(0),_contactPast(1),_phasePast(0),_phasePast(1));
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0)
    {
        for (int i(0); i < 2; ++i)
        {
            if (_contact(i) == _contactPast(i)&&fabs(_phase(i)-_phasePast(i))<0.01)
            {
                _switchStatus(i) = 0;
            }
            else if(_switchStatus(i) != 0)
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    //printf("%d %d %f  ",_contact(0),_contact(1),_phase(0));

    phaseResult = _phase;
    contactResult = _contact;
}

float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT()
{
    return _period;
}

void WaveGenerator::calcWave(Vec2 &phase, VecInt2 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)
    {
        _passT = (double)(getSystemTime() - _startT) * 1e-6;
        for (int i(0); i < 2; ++i)
        {
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio)//stance
            {
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio;
            }
            else//swing
            {
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
        // printf("%f %f %f  ",_passT,_normalT(0),_normalT(1));
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        phase << 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        phase << 0.5, 0.5;
    }
}