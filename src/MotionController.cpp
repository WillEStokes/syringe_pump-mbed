#include "debug.h"
#include "mbed.h"
#include "MotionController.h"

/*! Constructor */
MotionController::MotionController(PinName stepPin) : 
    _stepPin(stepPin),
    _stepperInterruptCb(callback(this, &MotionController::_stepperInterrupt)) {

}

/*! Setting the main parameters */
void MotionController::configure(float steps, float stepsPerSec, float accel, float decel) {
	// D(printf("steps = %f \n", steps));
    // D(printf("stepsPerSec = %f \n", stepsPerSec));
    // D(printf("accel = %f \n", accel));
    // D(printf("decel = %f \n", decel));
    
    // Check range and store parameters
    _steps = (int) (steps + 0.5f); // Desired number of steps
    _speed = stepsPerSec;
    _accel = accel;
    _decel = decel;
    
    // Step pin
    _stepPin = 0;
        
}

int MotionController::getState() {
    return _state;
}

int MotionController::getStepsPerformed() {
    return _stepsPerformed;
}

int MotionController::getC() {
    return (int)(_c + 0.5f);
}

void MotionController::reset() {
    _stop = 1;
}

int MotionController::createMotionProfile() {
    float alpha = 1.0;
    _stepsPerformed = 0;
    // Starting state
    //_state = RAMP_UP;
    
    // Calculate required parameters
    _c0 = 1000000.0f * sqrt((2.0f * alpha) / _accel);
    // double _c0d = (1000000.0 * sqrt((2.0 * alpha) / _accel));
    //D(printf("c0 = %f \n c0d = %f \n", _c0, _c0d));
    // D(printf("c0 = %f \n", _c0));
    
    _n = 1;
    _c = _c0 * 0.676f; 
    // _accel_until potentially can overflow if acceleration is too small
    // Basically it is a step count value when desired speed is reached
    _max_s_lim = (_speed * _speed) / (2.0f * alpha * _accel);
    // D(printf("_accel_until = %d \n", _max_s_lim));
    // Calculate minimum 'c' value (when the maximum speed is reached)
    //_c_min = _c0 * (sqrt(_max_s_lim + 1.0) - sqrt((float)_max_s_lim));
    _c_min = (1.0f / _speed) * 1000000.0f;
    // D(printf("_c_min = %f \n", _c_min));
    
    // debug
    /*double _test_max_slim = (_speed * _speed) / (2.0 * alpha * _accel);
    double _c_min_test = _c0d * (sqrt(_test_max_slim + 1.0) - sqrt(_test_max_slim));
    D(printf("_test_max_slim = %f \n _c_min_test = %f \n", _test_max_slim, _c_min_test));*/
    
    //_c_min = (int) (_c_min_test + 0.5);
    
    _accel_lim = (_steps * _decel) / (_accel + _decel);
    // D(printf("_accel_lim = %d \n", _accel_lim));
    
    if (_max_s_lim < _accel_lim) {
        _decel_n = -(_max_s_lim) * (_accel / _decel);
    } else {
        _decel_n = -(_steps - _accel_lim);
    }
    // D(printf("_decel_n = %d \n", _decel_n));
    
    _decel_start = _decel_n + _steps;
    // D(printf("_decel_start = %d \n", _decel_start));
    
    if (_c_min < 10) {
        _c_min = 10;
        return 0; // error, user wants stepping which is too fast for the controller
    } else {
        return 1;
    }
}

int MotionController::createMaxSpeedMotionProfile() {
    float alpha = 1.0;
    _stepsPerformed = 0;
    // Starting state
    //_state = RAMP_UP;
    
    // Calculate required parameters
    _c0 = 1000000.0f * sqrt((2.0f * alpha) / _accel);
    // double _c0d = (1000000.0 * sqrt((2.0 * alpha) / _accel));
    //D(printf("c0 = %f \n c0d = %f \n", _c0, _c0d));
    // D(printf("c0 = %f \n", _c0));
    
    _n = 1;
    _c = _c0 * 0.676f; 
    // _accel_until potentially can overflow if acceleration is too small
    // Basically it is a step count value when desired speed is reached
    _max_s_lim = (_speed * _speed) / (2.0f * alpha * _accel);
    // D(printf("_accel_until = %d \n", _max_s_lim));
    // Calculate minimum 'c' value (when the maximum speed is reached)
    //_c_min = _c0 * (sqrt(_max_s_lim + 1.0) - sqrt((float)_max_s_lim));
    _c_min = (1.0f / _speed) * 1000000.0f;
    
    if (_c_min < 10) _c_min = 10;
    
    // D(printf("_c_min = %f \n", _c_min));
    
    _steps = 2000000000;
    _decel_n = 1;
    _decel_start = 2147483647; 
    
    return 0;
}

void MotionController::_stepperInterrupt() {
    _stepPin = 1; // Enable step pin

    _stepsPerformed++; // Increment number of steps performed
    float new_c;
        
    if ((_stepsPerformed < _steps) && (_stop == 0)) {
        switch (_state) {
            case RAMP_UP:
                new_c = _c - (_c * 2) / (4 * _n + 1);
                
                if (_stepsPerformed >= _decel_start) {
                    // Go to decel state
                    _state = RAMP_DOWN;
                    _n = _decel_n;
                //} else if (_stepsPerformed >= _max_s_lim) {
                } else if (new_c <= _c_min) {
                    _state = RAMP_MAX;
                    new_c = _c_min;
                }
                
                if ((int)(new_c) != (int)(_c)) {
                    _timer.attach_us(_stepperInterruptCb, (int)(new_c + 0.5f));
                    // std::chrono::duration<int, std::micro> delay((int)(new_c + 0.5f));
                    // _timer.attach(_stepperInterruptCb, delay);
                }
                
                _c = new_c;
                break;
            case RAMP_MAX:
                            
                if (_stepsPerformed >= _decel_start) {
                    _state = RAMP_DOWN;
                    _n = _decel_n;
                }                    
                
                break;
            
            case RAMP_DOWN:
                new_c = _c - (_c * 2) / (4 * _n + 1);
                
                if ((int)(new_c) != (int)(_c)) {
                    _timer.attach_us(_stepperInterruptCb, (int)(new_c + 0.5f));
                    // std::chrono::duration<int, std::micro> delay((int)(new_c + 0.5f));
                    // _timer.attach(_stepperInterruptCb, delay);
                }
                
                _c = new_c;
            
                break;
            
        } 
        
        _n++;
    
    } else {
        _timer.detach();
        if (_stop == 0) callbackPumpingDone.call();
    }

    _stepPin = 0; // Disable step pin
}

void MotionController::run() {
    _stop = 0;
    // Starting state
    _state = RAMP_UP;
    // Start timer
    _timer.attach_us(_stepperInterruptCb, (int)(_c + 0.5f));
    // std::chrono::duration<int, std::micro> delay((int)(_c + 0.5f));
    // _timer.attach(_stepperInterruptCb, delay);
}

