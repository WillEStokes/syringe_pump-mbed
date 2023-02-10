#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

class PIDImpl
{
    public:      
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        void setPID( double dt, double max, double min, double Kp, double Kd, double Ki );
        PID::PidData getPID();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double error;
        double Pout;
        double Iout;
        double derivative;
        double Dout;
        double output;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pidimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}

double PID::calculate( double setpoint, double pv )
{
    return pidimpl->calculate(setpoint,pv);
}

void PID::setPID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pidimpl->setPID(dt, max, min, Kp, Kd, Ki);
}

PID::PidData PID::getPID()
{
    static PID::PidData pidData;

    pidData = pidimpl->getPID();

    return pidData;
}

PID::~PID() 
{
    delete pidimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

void PIDImpl::setPID( double dt, double max, double min, double Kp, double Kd, double Ki ) {
    _dt = dt;
    _max = max;
    _min = min;
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _pre_error = 0;
    _integral = 0;
}

PID::PidData PIDImpl::getPID() {
    static PID::PidData pidImplData;
    pidImplData.dt = _dt;
    pidImplData.max = _max;
    pidImplData.min = _min;
    pidImplData.Kp = _Kp;
    pidImplData.Kd = _Kd;
    pidImplData.Ki = _Ki;

    return pidImplData;
}

double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    error = setpoint - pv;

    // Proportional term
    Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    Iout = _Ki * _integral;

    // Derivative term
    derivative = (error - _pre_error) / _dt;
    Dout = _Kd * derivative;

    // Calculate total output
    output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
