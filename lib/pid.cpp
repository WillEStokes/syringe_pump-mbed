#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

// PID::~PID() 
// {
// }

PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki, double Kf, float step, float setpoint, uint8_t method, PinName mosfetPWM ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _Kf(Kf),
    _step(step),
    _setpoint(setpoint),
    _method(method),
    _pwmDutyCycle(mosfetPWM),
    _pre_error(0),
    _integral(0),
    _limit(1)
{
    _pwmDutyCycle.period_ms(100);
    _pwmDutyCycle.write(0);
}

void PID::setPID( double dt, double max, double min, double Kp, double Kd, double Ki, double Kf ) {
    _dt = dt;
    _max = max;
    _min = min;
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _Kf = Kf;
}

void PID::setPidMethod(uint8_t method) {
    _method = method;
}

void PID::setStep (float step) {
    _step = step;
}

void PID::setLimit (float limit) {
    _limit = limit;
}

void PID::reset() {
    _integral = 0;
    _setpoint = 0;
    _pwmDutyCycle.write(0);
}

double PID::getIntegral() {
    return _integral;
}

float PID::getSetpoint() {
    return _setpoint;
}

float PID::getStep() {
    return _step;
}

uint8_t PID::getMethod() {
    return _method;
}

float PID::getLimit() {
    return _limit;
}

PID::PidData PID::getPID() {
    static PID::PidData pidData;
    pidData.dt = _dt;
    pidData.max = _max;
    pidData.min = _min;
    pidData.Kp = _Kp;
    pidData.Kd = _Kd;
    pidData.Ki = _Ki;
    pidData.Kf = _Kf;

    return pidData;
}

double PID::calculate( float setpointTarget, float pv, bool initSetpoint)
{
    if (initSetpoint) // Initialise _setpoint to measured temp on first call
        _setpoint = pv;
    
    // Setpoint ramping
    if (_setpoint + _step < setpointTarget)
        _setpoint = _setpoint + _step;
    else if (_setpoint - _step >= setpointTarget)
        _setpoint = _setpoint - _step;
    else if ((_setpoint - _step < setpointTarget) || (_setpoint + _step > setpointTarget))
        _setpoint = setpointTarget;
    
    // Calculate error
    _error = _setpoint - pv;

    // Proportional term
    _Pout = _Kp * _error;

    // Integral term
    _temp_integral = _integral + _error * _dt;
    
    _Iout = _Ki * _temp_integral;

    // Derivative term
    _derivative = (_error - _pre_error) / _dt;
    _Dout = _Kd * _derivative;

    // Feedforward term
    _Fout = _Kf * _setpoint;

    // Calculate total output
    _output = _Pout + _Iout + _Dout + _Fout;

    // Implement clamping
    switch (_method) {
        case NONE:
            _integral = _temp_integral;
        break;
        case CLAMPING:
            if( _output > _max * _limit) {
                if (_error < 0)
                    _integral = _temp_integral;
            } else if( _output < _min ) {
                if (_error > 0)
                    _integral = _temp_integral;
            }
        break;
    }

    if( _output > _max * _limit) {
        _output = _max * _limit;
    } else if( _output < _min ) {
        _output = _min;
    }

    // Save error to previous error
    _pre_error = _error;

    _pwmDutyCycle = _output / (_max - _min);

    return _output;
}

#endif
