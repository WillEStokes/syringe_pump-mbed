#include "debug.h"
#include "SyringePump.h"
#include <string.h>
#include <math.h>  

const double PI = 3.141592653589793;

/*! Initialise list of responding functions */
const SyringePump::ComMessage SyringePump::comMessages[] = {
    {FID_GET_STATUS, (SyringePump::messageHandlerFunc)&SyringePump::getStatus},
    {FID_STOP_PUMP, (SyringePump::messageHandlerFunc)&SyringePump::stopPump},
    {FID_START_PUMP, (SyringePump::messageHandlerFunc)&SyringePump::startPump},
    {FID_SET_HARDWARE_CONFIG, (SyringePump::messageHandlerFunc)&SyringePump::setHardwareConfig},
    {FID_SET_FLOW_CONFIG, (SyringePump::messageHandlerFunc)&SyringePump::setFlowConfig},
    {FID_GET_HARDWARE_CONFIG, (SyringePump::messageHandlerFunc)&SyringePump::getHardwareConfig},
    {FID_MAX_PULL, (SyringePump::messageHandlerFunc)&SyringePump::maxPull},
    {FID_MAX_PUSH, (SyringePump::messageHandlerFunc)&SyringePump::maxPush},
    {FID_DISABLE_MOTOR_HOLD, (SyringePump::messageHandlerFunc)&SyringePump::disableMotorHold},
    {FID_GET_STEPDRV_ERROR, (SyringePump::messageHandlerFunc)&SyringePump::getStepDrvErrorId},
    {FID_GET_FLOW_CONFIG, (SyringePump::messageHandlerFunc)&SyringePump::getFlowConfig},
    {FID_RESET_PUMP, (SyringePump::messageHandlerFunc)&SyringePump::resetPump},
    {FID_GET_PUMP_ERROR, (SyringePump::messageHandlerFunc)&SyringePump::getPumpErrorId},
    {FID_GET_SYS_INFO, (SyringePump::messageHandlerFunc)&SyringePump::getSysInfo},
    {FID_IDENTIFY_ITSELF, (SyringePump::messageHandlerFunc)&SyringePump::identifyItself}
};

/*! Parameterized constructor */
SyringePump::SyringePump(
        PinName mosi,
        PinName miso,
        PinName sclk,
        PinName ss,
        PinName dirPin,
        PinName stepPin,
        PinName maxLimSwPin,
        PinName minLimSwPin,
        PinName greenLED,
        PinName yellowLED,
        PinName redLED,
        PinName stepperErrorPin,
        PinName stepperResetPin,
        PinName slaPin)
        : _stepperDriver(mosi, miso, sclk, ss),
        _motionController(stepPin),
        _maxLimSwPin(maxLimSwPin),
        _minLimSwPin(minLimSwPin),
        _stepperErrorPin(stepperErrorPin),
        _greenLED(greenLED),
        _yellowLED(yellowLED),
        _redLED(redLED),
        _dirPin(dirPin),
        _stepperResetPin(stepperResetPin),
        _slaPin(slaPin),
        _fidCount(sizeof (comMessages) / sizeof (ComMessage)), // constant
        _msgHeaderLength(sizeof (MessageHeader)) { // constant
    // add additional code to execute during the construction
    // Turn LEDs OFF by default
    _yellowLED = 0;
    _greenLED = 0;
    _redLED = 1;
    // D(printf("Number of FIDs: %d \n", _fidCount));
    // Initialise non-constant variables
    _flowConfigured = false;
    
    _hardwareConfig = new HardwareConfig;            
    _flowConfig = new FlowConfig;
    _pumpErrorList = new PumpErrorList;
            
    _pumpErrorList->maxLimitSwitchActive = 0;
    _pumpErrorList->minLimitSwitchActive = 0;
    _pumpErrorList->stepperDriverError = 0;
    _pumpErrorList->stepperDriverNotConfigured = 0;
            
    _pumpError = 0;
            
    _stepperResetPin = 0;
}

/* Implementation of FIDs
 * (functions received through ethernet)
 */

/*! Get status */
void SyringePump::getStatus(const MessageHeader* data) { 
    static SystemStatus status; // static is needed to avoid memory allocation every time the function is called
    
    status.header.packetLength = sizeof(SystemStatus);
    status.header.fid = FID_GET_STATUS;
    
    status.pumpState = _pumpState;
    status.pumpError = _pumpError;
    
    status.suppliedVolume_ml = _motionController.getStepsPerformed() / _stepsPer_ml;
    if (_pumpState == PUMP_RUNNING) {
        status.flowRate_mlmin = ((1000000.0f / _motionController.getC()) / _stepsPer_ml) * 60.0f;
    } else {
        status.flowRate_mlmin = 0.0f;
    }
    
    _socket->send((char*) &status, sizeof(SystemStatus));
}

/*! Get system info */
void SyringePump::getSysInfo(const MessageHeader* data) {
    static SystemInfo systemInfo;
    
    systemInfo.header.packetLength = sizeof(SystemInfo);
    systemInfo.header.fid = FID_GET_SYS_INFO;
    
    strcpy(systemInfo.fwVersion, FW_VERSION);
    strcpy(systemInfo.pumpId, PUMP_ID);
    strcpy(systemInfo.ipAddr, _ipAddr.get_ip_address());
    strcpy(systemInfo.macAddr, _macAddr);
    
    _socket->send((char*) &systemInfo, sizeof(SystemInfo));
}

/*! Identify itself */
void SyringePump::identifyItself(const MessageHeader* data) {
    int prevPumpState = _pumpState;
    _tickerGreenLED.attach(callback(this, &SyringePump::flipGreenLED), 100ms);
    // wait(1.2);
    setPumpState(prevPumpState);
    comReturn(data, MSG_OK);
}

/*! Stop pump */
void SyringePump::stopPump(const MessageHeader* data) {    
    // D(printf("stopPump command received\n"));
    
    disablePump();
    
    setPumpState(IDLE, true);
    // If everything went OK
    comReturn(data, MSG_OK);
}

/*! Start pump */
void SyringePump::startPump(const MessageHeader* data) {    
    // D(printf("Starting Pump\n"));
    
    if (!_flowConfigured) {
        // Flow not configured
        comReturn(data, MSG_ERROR_FLOW_NOT_CONFIGURED);
        return;
    }
    //! Hardware must be already configured in advance
        
    // Check for limit switches and direction (0 = pull, 1 = push)
    if (((_maxLimSwPin == 0) && (_flowConfig->direction == 1))
        || ((_minLimSwPin == 0) && (_flowConfig->direction == 0))) {
        comReturn(data, MSG_ERROR_LIMIT_SW_ACTIVE);
        return;
    }
    
    if (_pumpErrorList->stepperDriverError == 1) { // Error in the stepper driver
        comReturn(data, MSG_ERROR_STEPDRV_ERR);
        return;
    }
    
    // Calculate syringe area
    float syringeArea_mm2 = PI * pow((_flowConfig->syringeDiameter_mm / 2.0f), 2);
    // D(printf("Syringe area = %f \n", syringeArea_mm2));
    // Total steps per revolution
    float stepsPerRev = _hardwareConfig->stepMode * _hardwareConfig->stepsPerRev;
    // Calculate steps/ml
    float stepsPer_ml = ((1000.0f / syringeArea_mm2) * stepsPerRev) / _hardwareConfig->leadScrewPitch_mm;
    _stepsPer_ml = stepsPer_ml;
    // Calculate total steps required
    float steps = (_flowConfig->desVolume_ml * stepsPer_ml); 
    float stepsPerSec = (_flowConfig->desFlowrate_mlpmin / 60.0f * stepsPer_ml);
    
    // Set constant acceleration and deceleration (steps/s)
    // must be based on the microstepping mode 
    float accel = _hardwareConfig->pumpAcc_RevPerSecSec * stepsPerRev; // converting rev/s^2 to steps/s^2
    float decel = _hardwareConfig->pumpDec_RevPerSecSec * stepsPerRev; // converting rev/s^2 to steps/s^2
    
    // Configure motion profile
    _motionController.configure(steps, stepsPerSec, accel, decel);
    // Create motion profile
    if (_motionController.createMotionProfile()) {
        // Motion profile created successfully
        _stepperDriver.enableDriver();
    
        _motionController.run();
        
        // Indicate state of a system
        setPumpState(PUMP_RUNNING);

        comReturn(data, MSG_OK);
    } else {
        // Error in creating motion profile (occurs when user requests to switch way too fast)
        comReturn(data, MSG_ERROR_SWITCHING_OVER_MAX);
    }
}

/*! Configure hardware */
void SyringePump::setHardwareConfig(const SetHardwareConfig* data) {
    
    // Debug info
    // D(printf("SetPumpConfig command received\n"));
    // D(printf("stepMode = %d\n", data->hardwareConfig.stepMode));
    // D(printf("CurrentMilliamps = %d\n", data->hardwareConfig.maxDriverCurrent_mA));
    // D(printf("stepsPerRev = %d\n", data->hardwareConfig.stepsPerRev));
    // D(printf("leadScrewPitch_mm = %f\n", data->hardwareConfig.leadScrewPitch_mm));
    // D(printf("maxPullPushAcc_RevPerSecSec = %f \n", data->hardwareConfig.maxPullPushAcc_RevPerSecSec));
    // D(printf("maxPullPushVel_RevPerSec = %f \n", data->hardwareConfig.maxPullPushVel_RevPerSec));
    // D(printf("pumpAcc_RevPerSecSec = %f \n", data->hardwareConfig.pumpAcc_RevPerSecSec));
    // D(printf("pumpDec_RevPerSecSec = %f \n", data->hardwareConfig.pumpDec_RevPerSecSec));
         
    // Do some checks
    if (((data->hardwareConfig.maxDriverCurrent_mA > 3000) || (data->hardwareConfig.maxDriverCurrent_mA < 132))
        || (data->hardwareConfig.leadScrewPitch_mm <= 0) || (data->hardwareConfig.leadScrewPitch_mm >= 10)
        || (data->hardwareConfig.stepsPerRev <= 0) || (data->hardwareConfig.stepsPerRev > 1000)
        || ((data->hardwareConfig.pwmFrequency != 0) && (data->hardwareConfig.pwmFrequency != 1))
        || ((data->hardwareConfig.pwmJitter != 0) && (data->hardwareConfig.pwmJitter != 1))
        || (data->hardwareConfig.pwmSlope > 3) // usigned int
        || (data->hardwareConfig.maxPullPushAcc_RevPerSecSec > 10) || (data->hardwareConfig.maxPullPushAcc_RevPerSecSec <= 0)
        || (data->hardwareConfig.maxPullPushVel_RevPerSec > 10) || (data->hardwareConfig.maxPullPushVel_RevPerSec <= 0)
        || (data->hardwareConfig.pumpAcc_RevPerSecSec > 10) || (data->hardwareConfig.pumpAcc_RevPerSecSec <= 0)
        || (data->hardwareConfig.pumpDec_RevPerSecSec > 10) || (data->hardwareConfig.pumpDec_RevPerSecSec <= 0)) {
        comReturn(data, MSG_ERROR_INVALID_PARAMETER);
        return;
    }

    switch (data->hardwareConfig.stepMode) {
        case 128:
        case 64:
        case 32:
        case 16:
        case 8:
        case 4:
        case 2:
        case 1:
            break;
        default:
            comReturn(data, MSG_ERROR_INVALID_PARAMETER);
            return;
    }
            
    // Hardware config checked, save it
    memcpy(_hardwareConfig, &data->hardwareConfig, sizeof(HardwareConfig));
    
    // Apply hardware config to the stepper driver
    applyHardwareConfig();
    
    if (_pumpErrorList->stepperDriverNotConfigured) {
        comReturn(data, MSG_ERROR_STEPDRV_NOT_CONFIGURED);
    } else {
        comReturn(data, MSG_OK);
    }
}

/*! Configure flow */
void SyringePump::setFlowConfig(const SetFlowConfig* data) {

    // Debug information
    // D(printf("syringeDiameter_mm = %f\n", data->flowConfig.syringeDiameter_mm));
    // D(printf("stepMode from saved config = %d\n", _hardwareConfig->stepMode));
    // D(printf("CurrentMilliamps from saved config = %d\n", _hardwareConfig->maxDriverCurrent_mA));
    
    if  ((data->flowConfig.desFlowrate_mlpmin <= 0) || (data->flowConfig.desFlowrate_mlpmin > 100)
        || (data->flowConfig.desVolume_ml <= 0) || ( data->flowConfig.desVolume_ml > 200)
        || (data->flowConfig.syringeDiameter_mm <= 0) || (data->flowConfig.syringeDiameter_mm > 100)
        || ((data->flowConfig.direction != 0) && (data->flowConfig.direction != 1))) {
        // One of the parameters is invalid
        comReturn(data, MSG_ERROR_INVALID_PARAMETER);
        return;
    }
            
    // Reach here if everything went OK
    memcpy(_flowConfig, &data->flowConfig, sizeof(FlowConfig));
    
    // Apply hardware config to set the direction correctly
    applyHardwareConfig();
    
    // Set global variable
    setFlowConfigured(true);
    
    if (_pumpErrorList->stepperDriverNotConfigured) {
        comReturn(data, MSG_ERROR_STEPDRV_NOT_CONFIGURED);
    } else {
        comReturn(data, MSG_OK);
    }
    
}

void SyringePump::getHardwareConfig(const MessageHeader* data) {
    static GetHardwareConfig hwConfig; // static is needed to avoid memory allocation every time the function is called
    
    hwConfig.header.packetLength = sizeof(GetHardwareConfig);
    hwConfig.header.fid = FID_GET_HARDWARE_CONFIG;
    
    memcpy(&hwConfig.hardwareConfig, _hardwareConfig, sizeof(HardwareConfig));
    
    _socket->send((char*) &hwConfig, sizeof(GetHardwareConfig));
}

void SyringePump::getFlowConfig(const MessageHeader* data) {
    static GetFlowConfig flConfig; // static is needed to avoid memory allocation every time the function is called
    
    flConfig.header.packetLength = sizeof(GetFlowConfig);
    flConfig.header.fid = FID_GET_FLOW_CONFIG;
    
    memcpy(&flConfig.flowConfig, _flowConfig, sizeof(FlowConfig));
    
    _socket->send((char*) &flConfig, sizeof(GetFlowConfig));
}

void SyringePump::maxPull(const MessageHeader* data) {
    // Check for driver error
    if (_pumpErrorList->stepperDriverError == 1) { // Error in the stepper driver
        comReturn(data, MSG_ERROR_STEPDRV_ERR);
        return;
    }
    
    if (_pumpErrorList->stepperDriverNotConfigured == 1) { // Error in the stepper driver
        comReturn(data, MSG_ERROR_STEPDRV_NOT_CONFIGURED);
        return;
    }
    
    // Set direction (0 = pull, 1 = push)
    if (_minLimSwPin == 1) { // if minimum limit switch not pressed
        _stepperDriver.setDirection(0); // Pull syringe
    } else {
        comReturn(data, MSG_ERROR_LIMIT_SW_ACTIVE);
        return;
    }
    
    float stepsPerRev = _hardwareConfig->stepMode * _hardwareConfig->stepsPerRev;
    float accel = _hardwareConfig->maxPullPushAcc_RevPerSecSec * stepsPerRev; // converting rev/s^2 to steps/s^2
    float decel = accel;
    
     // Configure motion profile
    _motionController.configure(0, stepsPerRev * _hardwareConfig->maxPullPushVel_RevPerSec, accel, decel);
    // Create motion profile
    _motionController.createMaxSpeedMotionProfile();
    
    _stepperDriver.enableDriver();
    
    _motionController.run();
    
    // Indicate state of a system
    setPumpState(PUMP_RUNNING);

    comReturn(data, MSG_OK);
}
void SyringePump::maxPush(const MessageHeader* data) {
    // Check for driver error
    if (_pumpErrorList->stepperDriverError == 1) { // Error in the stepper driver
        comReturn(data, MSG_ERROR_STEPDRV_ERR);
        return;
    }
    
    if (_pumpErrorList->stepperDriverNotConfigured == 1) { // Error in the stepper driver
        comReturn(data, MSG_ERROR_STEPDRV_NOT_CONFIGURED);
        return;
    }
    
    // Set direction (0 = pull, 1 = push)
    if (_maxLimSwPin == 1) { // if maximum limit switch not pressed
        _stepperDriver.setDirection(1); // Push syringe
    } else {
        comReturn(data, MSG_ERROR_LIMIT_SW_ACTIVE);
        return;
    }
    
    float stepsPerRev = _hardwareConfig->stepMode * _hardwareConfig->stepsPerRev;
    float accel = _hardwareConfig->maxPullPushAcc_RevPerSecSec * stepsPerRev; // converting rev/s^2 to steps/s^2
    float decel = accel;
    
    // Configure motion profile
    _motionController.configure(0, stepsPerRev * _hardwareConfig->maxPullPushVel_RevPerSec, accel, decel);
    // Create motion profile
    _motionController.createMaxSpeedMotionProfile();
    
    _stepperDriver.enableDriver();
    
    _motionController.run();
    
    // Indicate state of a system
    setPumpState(PUMP_RUNNING);

    comReturn(data, MSG_OK);
}

void SyringePump::disableMotorHold(const MessageHeader* data) {
    _stepperDriver.disableDriver();
    comReturn(data, MSG_OK);
}

void SyringePump::resetPump(const MessageHeader* data) {
    disablePump();
    
    // Reset all errors
    unsetPumpError(PUMP_MAXLIM);
    unsetPumpError(PUMP_MINLIM);
    unsetPumpError(PUMP_DRIVER_ERROR);
    unsetPumpError(PUMP_STEPDRV_NOT_CONFIGURED);
    // Hardware reset
    _stepperResetPin = 1;
    // wait(0.1);
    _stepperResetPin = 0;
    
     // Reinitialise hardware (reset driver and initialise to default settings)
    initHardware();
    
     // Indicate state of a system
    setPumpState(IDLE);
    
    // Check if limit switches are already pressed
    if (_maxLimSwPin == 0) { // Active low
       setPumpError(PUMP_MAXLIM); 
    } else if (_minLimSwPin == 0) {
       setPumpError(PUMP_MINLIM);
    }
    
    // Check if there is a stepper driver error
    if (_stepperErrorPin == 0) {
        setPumpError(PUMP_DRIVER_ERROR);
    }
    
    comReturn(data, MSG_OK);
}

void SyringePump::getStepDrvErrorId(const MessageHeader* data) {
    // Check if we can communicate to the stepper driver

    uint16_t SR0 = _stepperDriver.readNonLatchedStatusFlags();
    uint16_t SR2_SR1 = _stepperDriver.readLatchedStatusFlagsAndClear();
    
    static GetStepperDriverError stepperDriverError; // static is needed to avoid memory allocation every time the function is called
    
    stepperDriverError.header.packetLength = sizeof(GetStepperDriverError);
    stepperDriverError.header.fid = FID_GET_STEPDRV_ERROR;
    
    // Check for every possible error
    // SR0
    stepperDriverError.OPENY = (SR0 & AMIS30543::OPENY) > 0 ? 1 : 0;
    stepperDriverError.OPENX = (SR0 & AMIS30543::OPENX) > 0 ? 1 : 0;
    stepperDriverError.WD = (SR0 & AMIS30543::WD) > 0 ? 1 : 0;
    stepperDriverError.CPFAIL = (SR0 & AMIS30543::CPFAIL) > 0 ? 1 : 0;
    stepperDriverError.TW = (SR0 & AMIS30543::TW) > 0 ? 1 : 0;
    // SR1 and SR2
    stepperDriverError.OVCXNB = (SR2_SR1 & AMIS30543::OVCXNB) > 0 ? 1 : 0;
    stepperDriverError.OVCXNT = (SR2_SR1 & AMIS30543::OVCXNT) > 0 ? 1 : 0;
    stepperDriverError.OVCXPB = (SR2_SR1 & AMIS30543::OVCXPB) > 0 ? 1 : 0;
    stepperDriverError.OVCXPT = (SR2_SR1 & AMIS30543::OVCXPT) > 0 ? 1 : 0;
    stepperDriverError.TSD = (SR2_SR1 & AMIS30543::TSD) > 0 ? 1 : 0;
    stepperDriverError.OVCYNB = (SR2_SR1 & AMIS30543::OVCYNB) > 0 ? 1 : 0;
    stepperDriverError.OVCYNT = (SR2_SR1 & AMIS30543::OVCYNT) > 0 ? 1 : 0;
    stepperDriverError.OVCYPB = (SR2_SR1 & AMIS30543::OVCYPB) > 0 ? 1 : 0;
    stepperDriverError.OVCYPT = (SR2_SR1 & AMIS30543::OVCYPT) > 0 ? 1 : 0;
  
    _socket->send((char*) &stepperDriverError, sizeof(GetStepperDriverError)); 
}

void SyringePump::getPumpErrorId(const MessageHeader* data) { 
    static GetPumpError pumpError; // static is needed to avoid memory allocation every time the function is called
    
    pumpError.header.packetLength = sizeof(GetPumpError);
    pumpError.header.fid = FID_GET_PUMP_ERROR;
    
    memcpy(&pumpError.pumpErrors, _pumpErrorList, sizeof(PumpErrorList));
  
    _socket->send((char*) &pumpError, sizeof(GetPumpError)); 
}

/* End of implementation
 * of FIDs
 */

/*! LED Functions */
void SyringePump::flipYellowLED() {
    _yellowLED = !_yellowLED;
}

void SyringePump::flipGreenLED() {
    _greenLED = !_greenLED;
    _redLED = !_redLED;
}

/*! Setting the pump state */
void SyringePump::setPumpState(int state, bool calledFromIRQ) {
    
    if (!calledFromIRQ) __disable_irq();    // Disable Interrupts if not called from an interrupt
    
    _pumpState = state;
    
    if (_pumpError == 0) {
        _yellowLED = 0;
    }
    
    switch(_pumpState) {
        case SYS_INIT:
            _greenLED = 0;
            _redLED = 1;
            break;
        case WAIT_FOR_CONNECTION:
            // Blink green LED
            _tickerGreenLED.attach(callback(this, &SyringePump::flipGreenLED), 500ms);
            break;
        case IDLE:
            // Solid green LED
            _tickerGreenLED.detach();
            _greenLED = 1;
            _redLED = 0;
            break;
        case PUMP_RUNNING:
            _tickerGreenLED.detach();
            _greenLED = 1;
            _redLED = 0;
            // solid yellow LED (only if there are no errors)
            if (_pumpError == 0) {
                _tickerYellowLED.detach();
                _yellowLED = 1;
            }
            break;
        default:
            _tickerGreenLED.detach();
            _greenLED = 0;
            _redLED = 1;
            break;
    }
    
    if (!calledFromIRQ) __enable_irq();     // Enable Interrupts if not called from an interrupt
}

/*! Setting the pump error */
void SyringePump::setPumpError(int error, bool calledFromIRQ) {
    
    if (!calledFromIRQ) __disable_irq(); // Disable Interrupts if not called from an interrupt
    
    _pumpError = 1;
    // Indicate error with a blinking LED
    _tickerYellowLED.attach(callback(this, &SyringePump::flipYellowLED), 250ms);
    
    switch (error) {
        case PUMP_MAXLIM:
            _pumpErrorList->maxLimitSwitchActive = 1;
            break;
        case PUMP_MINLIM:
            _pumpErrorList->minLimitSwitchActive = 1;
            break;
        case PUMP_DRIVER_ERROR:
            _pumpErrorList->stepperDriverError = 1;
            break;
        case PUMP_STEPDRV_NOT_CONFIGURED:
            _pumpErrorList->stepperDriverNotConfigured = 1;
            break;
        default:
            break;
    }
        
    if (!calledFromIRQ) __enable_irq(); // Enable Interrupts if not called from an interrupt
    
}

/*! Unsetting the pump error */
void SyringePump::unsetPumpError(int error, bool calledFromIRQ) {
    
    if (!calledFromIRQ) __disable_irq(); // Disable Interrupts if not called from an interrupt

    // Unset all errors except for the stepper driver error, it will require user to reset the pump
    switch (error) {
        case PUMP_MAXLIM:
            _pumpErrorList->maxLimitSwitchActive = 0;
            break;
        case PUMP_MINLIM:
            _pumpErrorList->minLimitSwitchActive = 0;
            break;
        case PUMP_STEPDRV_NOT_CONFIGURED:
            _pumpErrorList->stepperDriverNotConfigured = 0;
            break;
        case PUMP_DRIVER_ERROR:
            _pumpErrorList->stepperDriverError = 0;
            break;
        default:
            break;
    }
    
    if ((_pumpErrorList->maxLimitSwitchActive == 0) && (_pumpErrorList->minLimitSwitchActive == 0) 
        && (_pumpErrorList->stepperDriverError == 0) && (_pumpErrorList->stepperDriverNotConfigured ==0)) {
        _pumpError = 0;
        _tickerYellowLED.detach();
        // this is needed to make the yellow led on when while moving the limitswitch gets unpressed
        if (_pumpState == PUMP_RUNNING) {
            _yellowLED = 1;
        } else {
            _yellowLED = 0;
        }
    }            
       
    if (!calledFromIRQ) __enable_irq(); // Enable Interrupts if not called from an interrupt
    
}


void SyringePump::pumpingFinished() {
    // Disabling the pump
    disablePump(true);
    // This is callback function triggered from the interrupt
    setPumpState(IDLE, true);
}

void SyringePump::maxLimSwitchHit() { // PUMP ERROR
    // Disabling the pump
    disablePump(true);
    // Set the corresponding error flag
    setPumpError(PUMP_MAXLIM, true);
}

void SyringePump::minLimSwitchHit() { // PUMP ERROR
    // Disabling the pump
    disablePump(true);
    // Set the corresponding error flag
    setPumpError(PUMP_MINLIM, true);
}

void SyringePump::maxLimSwitchNoHit() { // PUMP ERROR FIXED
    unsetPumpError(PUMP_MAXLIM, true);
}

void SyringePump::minLimSwitchNoHit() { // PUMP ERROR FIXED
    unsetPumpError(PUMP_MINLIM, true);
}

void SyringePump::stepperDriverError() { // PUMP ERROR
    // Disabling the pump
    disablePump(true);
    setPumpError(PUMP_DRIVER_ERROR, true);
}

void SyringePump::disablePump(bool calledFromIRQ) {

    if (!calledFromIRQ) __disable_irq();
    
    _motionController.reset();

    setFlowConfigured(false, calledFromIRQ);
    
    if (!calledFromIRQ) __enable_irq();
}

/*! Setter for the _flowConfigured private member */
void SyringePump::setFlowConfigured(bool value, bool calledFromIRQ) {
    if (!calledFromIRQ) __disable_irq();
    _flowConfigured = value;
    if (!calledFromIRQ) __enable_irq();
}

/*! Applying hardware config */
void SyringePump::applyHardwareConfig() {
    // Applying settings to the stepper driver
 
    // PWM Frequency
    if (_hardwareConfig->pwmFrequency == 0) {
        //  D(printf("setting default pwm frequency (22.8 kHz) \n"));
        _stepperDriver.setPwmFrequencyDefault();
    } else {
        // D(printf("setting double pwm frequency (45.6 kHz) \n"));
        _stepperDriver.setPwmFrequencyDouble();
    }
    
    // PWM slope
    // D(printf("applying stepper motor pwm slope = %d \n", _hardwareConfig->pwmSlope));
    _stepperDriver.setPwmSlope(_hardwareConfig->pwmSlope);

    if (_hardwareConfig->pwmJitter == 1) {
        //  D(printf("enabling pwm jitter \n"));
        _stepperDriver.setPwmJitterOn();
    } else {
        // D(printf("disabling pwm jitter \n"));
        _stepperDriver.setPwmJitterOff();
    }
    // Direction
    // D(printf("applying stepper motor direction = %d \n", _flowConfig->direction));
    _stepperDriver.setDirection(_flowConfig->direction);
    
    // Stepping mode
    // D(printf("applying stepMode = %d \n", _hardwareConfig->stepMode));
    _stepperDriver.setStepMode(_hardwareConfig->stepMode);
    
    // Maximum current
    // D(printf("applying maxCurrentMilliamps = %d \n", _hardwareConfig->maxDriverCurrent_mA));
    _stepperDriver.setCurrentMilliamps(_hardwareConfig->maxDriverCurrent_mA);
    
    // Verify settings and flag if it wasn't successful
    if (!_stepperDriver.verifySettings()) {
        setPumpError(PUMP_STEPDRV_NOT_CONFIGURED);
    } else if (_pumpErrorList->stepperDriverNotConfigured == 1) {
        // if previously there was an error, undo it
        unsetPumpError(PUMP_STEPDRV_NOT_CONFIGURED);
    }

}

/*! Initialising Hardware */
void SyringePump::initHardware() {
    // Reset the stepper driver
    _stepperDriver.resetSettings();
    // Set sla gain to 0.5
    _stepperDriver.setSlaGainDefault();
    
    _hardwareConfig->pwmFrequency = 0; // 0 = default (22.8 kHz), 1 = double (45.6 kHz)
    _hardwareConfig->pwmSlope = 0; // 0 = 200 V/us, 1 = 140 V/us, 2 = 70 v/us, 3 = 35 V/us
    _hardwareConfig->pwmJitter = 0; // 0 = OFF, 1 = ON
    
    // Default parameters, can be changed through API
    _hardwareConfig->leadScrewPitch_mm = 1.5f;
    _hardwareConfig->maxDriverCurrent_mA = 850;
    _hardwareConfig->stepMode = 32;
    _hardwareConfig->stepsPerRev = 400;
    // Direction of the flow (0 = pull, 1 = push)
    _flowConfig->direction = 0;
    
    _hardwareConfig->maxPullPushAcc_RevPerSecSec = 4.0f; // Acceleration rate for maximum speed pull/push (rev/s^2)
    _hardwareConfig->maxPullPushVel_RevPerSec = 4.0f; // Velocity when push/pull is at max (rev/s)
    _hardwareConfig->pumpAcc_RevPerSecSec = 0.1f; // Acceleration rate for normal pumping (rev/s^2)
    _hardwareConfig->pumpDec_RevPerSecSec= 0.1f; // Deceleration rate for normal puming (rev/s^2)
    
    applyHardwareConfig();
}

/*! Initialising Ethernet */
void SyringePump::initEthernet() {
    
    // TODO: Get these values from flash memory, static for now
    static const char* mbedIp = IP_ADDRESS; // IP
    static const char* mbedMask = NETW_MASK; // Mask
    static const char* mbedGateway = GATEAWAY; // Gateway 
    bool dhcp = false;
    
    // // DHCP?
    // if (dhcp) {
    //     _eth.connect();
    // } else {
    //     _eth.set_network(mbedIp, mbedMask, mbedGateway);
    //     _eth.connect();
    // }
    
    // // Getting mbed IP and MAC address
    // //const char* ip = _eth.get_ip_address();
    // //const char* mac = _eth.get_mac_address();
    // _ipAddr = _eth.get_ip_address();
    // _macAddr = _eth.get_mac_address();

    // Set static IP
    _eth.set_network(IP_ADDRESS, NETW_MASK, GATEAWAY);

    // Bring up the ethernet interface
    _eth.connect();

    // Show the network address
    _eth.get_ip_address(&_ipAddr);
    
    // D(printf("IP address is: %s\n", _ipAddr ? _ipAddr : "No IP"));
    // D(printf("MAC address is: %s\n", _macAddr ? _macAddr : "No MAC"));
    
    // Creating TCP server on Ethernet interface
    _server.open(&_eth);
    _server.bind(TCP_PORT);
    _server.listen(1);
    
    // Configuring TCP socket
    // _socket.set_blocking(true);
    // _socket.set_timeout(-1);
    _server.set_blocking(true);
    _server.set_timeout(-1);
}

/*! Getting a function pointer based on the FID */
const SyringePump::ComMessage* SyringePump::getComFromHeader(const MessageHeader* header) {

    if (header->fid >= _fidCount) { //Prevent getting out of an array
        return NULL;
    }

    return &comMessages[header->fid];
}

void SyringePump::comReturn(const void* data, const int errorCode) {
    MessageHeader *message = (MessageHeader*) data;
    message->packetLength = _msgHeaderLength;
    message->error = errorCode;
    _socket->send((char*) message, _msgHeaderLength);
}

/*! Main function */
void SyringePump::run() {   
    // Indicate initialising state of a system
    setPumpState(SYS_INIT);
 
    // Initialising hardware
    initHardware();
    
    // Motion controller's callback
    // _motionController.callbackPumpingDone.attach(this, &SyringePump::pumpingFinished);
    _motionController.callbackPumpingDone = mbed::callback(this, &SyringePump::pumpingFinished);
    
    // Limit switches interrupt setup
    _maxLimSwPin.mode(PullUp);
    _minLimSwPin.mode(PullUp);
    
    _maxLimSwPin.fall(callback(this, &SyringePump::maxLimSwitchHit));
    _minLimSwPin.fall(callback(this, &SyringePump::minLimSwitchHit));
    
    _maxLimSwPin.rise(callback(this, &SyringePump::maxLimSwitchNoHit));
    _minLimSwPin.rise(callback(this, &SyringePump::minLimSwitchNoHit));
    
    // Stepper Driver error interrupt setup
    _stepperErrorPin.mode(PullUp);
    _stepperErrorPin.fall(callback(this, &SyringePump::stepperDriverError));
    
    // Check if limit switches are already pressed
    if (_maxLimSwPin == 0) { // Active low
       setPumpError(PUMP_MAXLIM); 
    } else if (_minLimSwPin == 0) {
       setPumpError(PUMP_MINLIM);
    }
    
    // Check if there is a stepper driver error
    if (_stepperErrorPin == 0) {
        setPumpError(PUMP_DRIVER_ERROR);
    }
                
    // Indicate state of a system
    setPumpState(WAIT_FOR_CONNECTION);
        
    // D(printf("Initialising Ethernet Interface...\n"));
    initEthernet();
    // D(printf("Starting up...\n"));

    // Initialise data buffer (receive)
    char data[256];
    // Create pointer to header data
    MessageHeader* header;

    while (true) {
        // Indicate state of a system
        setPumpState(WAIT_FOR_CONNECTION);
        
        // // D(printf("\nWaiting for new connection...\n"));
        // _server.accept(&_socket, &_clientAddr);	
        // // D(printf("accept %s:%d\n", _clientAddr.get_ip_address(), _clientAddr.get_port()));
        _socket = _server.accept();
        _socket->getpeername(&_clientAddr);

        // Indicate the state of a system
        setPumpState(IDLE);
                
        while(true) {
            // Wait for a header
            // if (_socket.recv(data, _msgHeaderLength) <= 0) break;
            _socketBytes = _socket->recv(data, _msgHeaderLength);
            if (_socketBytes <= 0) break;

            header = (MessageHeader*)data;

            if (header->packetLength != _msgHeaderLength) {
                if (_socket->recv(data + _msgHeaderLength, header->packetLength - _msgHeaderLength) <= 0) break;
            }
                                
            const ComMessage* comMessage = getComFromHeader(header);
                    
            if(comMessage != NULL && comMessage->replyFunc != NULL) {
                // D(printf("FID to call: %d\n", comMessage->fid));
                // Allow only pump stop and status commands when pump is running
                // Fact: comMessage->fid is equivalent to (*comMessage).fid
                if ((_pumpState == PUMP_RUNNING) && (comMessage->fid != FID_STOP_PUMP) && (comMessage->fid != FID_GET_STATUS) &&  (_pumpError == 0)) {
                    comReturn(data, MSG_ERROR_PUMP_RUNNING);
                } else {
                    (this->*comMessage->replyFunc)((void*)data);
                }
            } else {
                comReturn(data, MSG_ERROR_NOT_SUPPORTED);
            }
        }			
        // Client disconnected        
        // D(printf("Client disconnected, stopping and resetting the pump\n"));
        
        // Stop the pump
        disablePump();
        
        // Disable AMIS30543 driver
        _stepperDriver.disableDriver();
        
        // Reinitialise hardware
        initHardware();
        
        _socket->close();
        // Indicate disconnected state
        setPumpState(WAIT_FOR_CONNECTION);
    }
}
