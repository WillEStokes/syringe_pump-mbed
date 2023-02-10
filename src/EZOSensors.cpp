#include "../debug.h"
#include "EZOSensors.h"
#include <string.h>
#include <math.h>

/*! Initialise list of responding functions */
const EZOSensors::ComMessage EZOSensors::comMessages[] = {
    {FID_GET_STATUS, (EZOSensors::messageHandlerFunc)&EZOSensors::getStatus},
    {FID_SEND_READ_CMD, (EZOSensors::messageHandlerFunc)&EZOSensors::sendReadCMD},
    {FID_GET_SENSOR_DATA, (EZOSensors::messageHandlerFunc)&EZOSensors::getSensorData},
    {FID_GET_SENSOR_INFO, (EZOSensors::messageHandlerFunc)&EZOSensors::getSensorInfo},
    {FID_GET_SENSOR_STATUS, (EZOSensors::messageHandlerFunc)&EZOSensors::getSensorStatus},
    {FID_GET_SYSTEM_INFO, (EZOSensors::messageHandlerFunc)&EZOSensors::getSystemInfo},
    {FID_SET_TEMP_SETPOINT, (EZOSensors::messageHandlerFunc)&EZOSensors::setTempSetpoint},
    {FID_SET_PID, (EZOSensors::messageHandlerFunc)&EZOSensors::setPID},
    {FID_GET_PID, (EZOSensors::messageHandlerFunc)&EZOSensors::getPID},
    {FID_RESET_PID, (EZOSensors::messageHandlerFunc)&EZOSensors::resetPID},
    {FID_SET_PID_STEP, (EZOSensors::messageHandlerFunc)&EZOSensors::setPidStep},
    {FID_SET_PID_METHOD, (EZOSensors::messageHandlerFunc)&EZOSensors::setPidMethod},
    {FID_SET_ADDRESS, (EZOSensors::messageHandlerFunc)&EZOSensors::setEZOAddress},
    {FID_SET_PID_LIMIT, (EZOSensors::messageHandlerFunc)&EZOSensors::setPidLimit},
    {FID_GET_PID_LIMIT, (EZOSensors::messageHandlerFunc)&EZOSensors::getPidLimit},
    {FID_CALIBRATE_LOW, (EZOSensors::messageHandlerFunc)&EZOSensors::calibrateLow},
    {FID_CALIBRATE_MID, (EZOSensors::messageHandlerFunc)&EZOSensors::calibrateMid},
    {FID_CALIBRATE_HIGH, (EZOSensors::messageHandlerFunc)&EZOSensors::calibrateHigh},
    {FID_FACTORY_RESET_EZO, (EZOSensors::messageHandlerFunc)&EZOSensors::resetEzo},
};

/*! Parameterised constructor */
EZOSensors::EZOSensors(
        PinName redLED,
        PinName statusLED )
        // PinName mosfetPWM )
        :
        _pid_1(0.5, 100, 0, 9, 0, 0.006, 0, 0.1, 0, 1, D10),
        _pid_2(0.5, 100, 0, 9, 0, 0.006, 0, 0.1, 0, 1, D11),
        _pid_3(0.5, 100, 0, 9, 0, 0.006, 0, 0.1, 0, 1, D12),
        _phSensor(I2C_SDA, I2C_SCL, PH_ADDRESS),
        _tempSensor_1(I2C_SDA, I2C_SCL, TEMP_ADDRESS_1),
        _tempSensor_2(I2C_SDA, I2C_SCL, TEMP_ADDRESS_2),
        _tempSensor_3(I2C_SDA, I2C_SCL, TEMP_ADDRESS_3),
        _redLED(redLED),
        _statusLED(statusLED),
        // _pwmDutyCycle(mosfetPWM),
        _fidCount(sizeof (comMessages) / sizeof (ComMessage)), // constant
        _msgHeaderLength(sizeof (MessageHeader)) { //
    
    // Turn LED ON by default
    _redLED = 0;
    _statusLED = 1;

    // _pwmDutyCycle.period_ms(100);
    // _pwmDutyCycle.write(0);

    //initialise non-constant variables
    // _pidMax = 100;
    // _pidMin = 0;
    _tempSetpoint = 0;
    _statState = 0;
    _phSensorConnected = false;
    _tempSensor_1Connected = false;
    _tempSensor_2Connected = false;
    _tempSensor_3Connected = false;
    _CMDRead = false;
    _readingPending = false;
    _initSetpoint = true;
}

/*! Get status */
void EZOSensors::getStatus(const MessageHeader* data) { 
    static SystemStatus status; // static is needed to avoid memory allocation every time the function is called
    
    status.header.packetLength = sizeof(SystemStatus);
    status.header.fid = FID_GET_STATUS;
    
    checkSensorConnection();
    
    if (!_tempSensor_1Connected || !_tempSensor_2Connected || !_tempSensor_3Connected) {
        status.header.error = MSG_ERROR_SENSOR_DISCONNECTED;
    } else {
        status.header.error = MSG_OK;
    }
    
    status.boardState = _boardState;
    status.tempSetpoint = _tempSetpoint;
    status.phSensorConnected = _phSensorConnected;
    status.tempSensor_1Connected = _tempSensor_1Connected;
    status.tempSensor_2Connected = _tempSensor_2Connected;
    status.tempSensor_3Connected = _tempSensor_3Connected;
    status.pid_1Output = _pid_1Output;
    status.pid_2Output = _pid_2Output;
    status.pid_3Output = _pid_3Output;
    status.pid_1Integral = _pid_1.getIntegral();
    status.pid_2Integral = _pid_2.getIntegral();
    status.pid_3Integral = _pid_3.getIntegral();
    status.pid_1Setpoint = _pid_1.getSetpoint();
    status.pid_2Setpoint = _pid_2.getSetpoint();
    status.pid_3Setpoint = _pid_3.getSetpoint();
    status.pid_1Step = _pid_1.getStep();
    status.pid_2Step = _pid_2.getStep();
    status.pid_3Step = _pid_3.getStep();
    status.pid_1Method = _pid_1.getMethod();
    status.pid_2Method = _pid_2.getMethod();
    status.pid_3Method = _pid_3.getMethod();
    
    _socket->send((char*) &status, sizeof(SystemStatus));
}

/*! Configure hardware */
void EZOSensors::setTempSetpoint(const SetTempSetpoint* data) {
    _tempSetpoint = data->tempSetpoint;
    _initSetpoint = true;
}

void EZOSensors::setPidMethod(const SetPidMethod* data) {
    _pid_1.setPidMethod(data->antiWindUpMethod);
    _pid_2.setPidMethod(data->antiWindUpMethod);
    _pid_3.setPidMethod(data->antiWindUpMethod);
}

void EZOSensors::setPidStep(const SetPidStep* data) {
    _pid_1.setStep(data->step);
    _pid_2.setStep(data->step);
    _pid_3.setStep(data->step);
}

void EZOSensors::setPidLimit(const PidLimit* data) {
    _pid_1.setLimit(data->limit);
    _pid_2.setLimit(data->limit);
    _pid_3.setLimit(data->limit);
}

/*! Get sensor data */
void EZOSensors::getPidLimit(const GetPidConfig* data) {
    static PidLimit pidLimit;

    switch (data->pid) {
        case 1:
            pidLimit.limit = _pid_1.getLimit();
        break;
        case 2:
            pidLimit.limit = _pid_2.getLimit();
        break;
        case 3:
            pidLimit.limit = _pid_3.getLimit();
        break;
    }

    pidLimit.header.packetLength = sizeof(pidLimit);
    pidLimit.header.fid = FID_GET_PID_LIMIT;
    _socket->send((char*) &pidLimit, sizeof(pidLimit));
}

void EZOSensors::resetPID(const MessageHeader* data) {
    _pid_1.reset();
    _pid_2.reset();
    _pid_3.reset();
}

/*! Configure PID */
void EZOSensors::setPID(const PidParams* data) {
    _pid_1.setPID(data->dt , data->max, data->min, data->Kp, data->Kd, data->Ki, data->Kf);
    _pid_2.setPID(data->dt , data->max, data->min, data->Kp, data->Kd, data->Ki, data->Kf);
    _pid_3.setPID(data->dt , data->max, data->min, data->Kp, data->Kd, data->Ki, data->Kf);
    // _pidMax = data->max;
    // _pidMin = data->min;
}

void EZOSensors::getPID(const GetPidConfig* data) {
    static PidParams pidParams;
    static PID::PidData pidData;

    switch (data->pid) {
        case 1:
            pidData = _pid_1.getPID();
        break;
        case 2:
            pidData = _pid_2.getPID();
        break;
        case 3:
            pidData = _pid_3.getPID();
        break;
    }

    pidParams.header.packetLength = sizeof(PidParams);
    pidParams.header.fid = FID_GET_PID;
    pidParams.dt = pidData.dt;
    pidParams.max = pidData.max;
    pidParams.min = pidData.min;
    pidParams.Kp = pidData.Kp;
    pidParams.Kd = pidData.Kd;
    pidParams.Ki = pidData.Ki;
    pidParams.Kf = pidData.Kf;

    _socket->send((char*) &pidParams, sizeof(pidParams));
}

void EZOSensors::setEZOAddress(const SetEzoAddress* data) {
    static int addressChanged = 2;
    
    switch (data->oldAddress << 1) {
        case TEMP_ADDRESS_1:
            addressChanged = _tempSensor_1.setAddress(data->newAddress);
        break;
        case TEMP_ADDRESS_2:
            addressChanged = _tempSensor_2.setAddress(data->newAddress);
        break;
        case TEMP_ADDRESS_3:
            addressChanged = _tempSensor_3.setAddress(data->newAddress);
        break;
        default:
        break;
    }

    if (addressChanged == 1)
        comReturn(data, MSG_OK);
    else
        comReturn(data, MSG_ERROR_NOT_SUPPORTED);
}

void EZOSensors::resetEzo(const MessageHeader* data) {
    _phSensor.factoryReset();
    _tempSensor_1.factoryReset();
    _tempSensor_2.factoryReset();
    _tempSensor_3.factoryReset();
}

void EZOSensors::calibrateLow(const MessageHeader* data) {
    static int calibrationError;
    
    calibrationError = _phSensor.calibratingLow();

    if (calibrationError == 1)
        comReturn(data, MSG_OK);
    else
        comReturn(data, MSG_ERROR_NOT_SUPPORTED);
}

void EZOSensors::calibrateMid(const MessageHeader* data) {
    static int calibrationError;
    
    calibrationError = _phSensor.calibratingMid();

    if (calibrationError == 1)
        comReturn(data, MSG_OK);
    else
        comReturn(data, MSG_ERROR_NOT_SUPPORTED);
}

void EZOSensors::calibrateHigh(const MessageHeader* data) {
    static int calibrationError;
    
    calibrationError = _phSensor.calibratingHigh();

    if (calibrationError == 1)
        comReturn(data, MSG_OK);
    else
        comReturn(data, MSG_ERROR_NOT_SUPPORTED);
}

/*! Send read cmd */
void EZOSensors::sendReadCMD(const MessageHeader* data) {
    if (_readingPending) {
        return;
    }
    
    _phSensor.sendReadCMD();
    _tempSensor_1.sendReadCMD();
    _tempSensor_2.sendReadCMD();
    _tempSensor_3.sendReadCMD();
    
    _tickerEZO.attach(callback(this, &EZOSensors::receiveReading), 1s);
    _readingPending = true;
}

void EZOSensors::receiveReading() {
    _tickerEZO.detach();
    
    _readingPending = false;
    _CMDRead = true;
}

/*! Get sensor data */
void EZOSensors::getSensorData(const MessageHeader* data) {
    static SensorData sensorData;

    if (!_readingPending) {
        _phReading = _phSensor.receiveReading();
        _temp_1Reading = _tempSensor_1.receiveReading();
        _temp_2Reading = _tempSensor_2.receiveReading();
        _temp_3Reading = _tempSensor_3.receiveReading();
    }

    _pid_1Output = _pid_1.calculate(_tempSetpoint, _temp_1Reading, _initSetpoint);
    _pid_2Output = _pid_2.calculate(_tempSetpoint, _temp_2Reading, _initSetpoint);
    _pid_3Output = _pid_3.calculate(_tempSetpoint, _temp_3Reading, _initSetpoint);
    if (_CMDRead)
        _initSetpoint = false;
    // _pwmDutyCycle = _pidOutput / (_pidMax - _pidMin);

    sensorData.header.packetLength = sizeof(SensorData);
    sensorData.header.fid = FID_GET_SENSOR_DATA;
    sensorData.ph = _phReading;
    sensorData.temp_1 = _temp_1Reading;
    sensorData.temp_2 = _temp_2Reading;
    sensorData.temp_3 = _temp_3Reading;
    
    _socket->send((char*) &sensorData, sizeof(SensorData));
}

/*! Get sensor info */
void EZOSensors::getSensorInfo(const MessageHeader* data) {
    static SensorInfo sensorInfo;
    
    sensorInfo.header.packetLength = sizeof(SensorInfo);
    sensorInfo.header.fid = FID_GET_SENSOR_INFO;
    
    if (_readingPending) {
        sensorInfo.header.error = MSG_ERROR_READING_PENDING;
        _socket->send((char*) &sensorInfo, sizeof(SensorInfo));
        return;
    }
    
    _phSensorInfo = _phSensor.getSensorInfo();
    _tempSensor_1Info = _tempSensor_1.getSensorInfo();
    _tempSensor_2Info = _tempSensor_2.getSensorInfo();
    _tempSensor_3Info = _tempSensor_3.getSensorInfo();
    
    strcpy(sensorInfo.ph, _phSensorInfo.c_str());
    strcpy(sensorInfo.temp_1, _tempSensor_1Info.c_str());
    strcpy(sensorInfo.temp_2, _tempSensor_2Info.c_str());
    strcpy(sensorInfo.temp_3, _tempSensor_3Info.c_str());
    
    _socket->send((char*) &sensorInfo, sizeof(SensorInfo));
}

/*! Get sensor info */
void EZOSensors::getSensorStatus(const MessageHeader* data) {
    static SensorStatus sensorStatus;
    
    sensorStatus.header.packetLength = sizeof(SensorStatus);
    sensorStatus.header.fid = FID_GET_SENSOR_STATUS;
    
    if (_readingPending) {
        sensorStatus.header.error = MSG_ERROR_READING_PENDING;
        _socket->send((char*) &sensorStatus, sizeof(SensorStatus));
        return;
    }
    
    _phSensorStatus = _phSensor.getSensorStatus();
    _tempSensor_1Status = _tempSensor_1.getSensorStatus();
    _tempSensor_2Status = _tempSensor_2.getSensorStatus();
    _tempSensor_3Status = _tempSensor_3.getSensorStatus();
    
    strcpy(sensorStatus.ph, _phSensorStatus.c_str());
    strcpy(sensorStatus.temp_1, _tempSensor_1Status.c_str());
    strcpy(sensorStatus.temp_2, _tempSensor_2Status.c_str());
    strcpy(sensorStatus.temp_3, _tempSensor_3Status.c_str());
    
    _socket->send((char*) &sensorStatus, sizeof(SensorStatus));
}

void EZOSensors::checkSensorConnection() {
    if (_CMDRead) {
        /*! Determine if sensors are connected here */
        if (_phReading != 0) {
            _phSensorConnected = true;
        } else {
            _phSensorConnected = false;
        }
        
        if (_temp_1Reading != -1023) {
            _tempSensor_1Connected = true;
        } else {
            _tempSensor_1Connected = false;
        }

        if (_temp_2Reading != -1023) {
            _tempSensor_2Connected = true;
        } else {
            _tempSensor_2Connected = false;
        }

        if (_temp_3Reading != -1023) {
            _tempSensor_3Connected = true;
        } else {
            _tempSensor_3Connected = false;
        }
    }
    
}

/*! Get system info */
void EZOSensors::getSystemInfo(const MessageHeader* data) {
    static SystemInfo systemInfo;
    
    systemInfo.header.packetLength = sizeof(SystemInfo);
    systemInfo.header.fid = FID_GET_SYSTEM_INFO;
    
    strcpy(systemInfo.fwVersion, FW_VERSION);
    strcpy(systemInfo.boardId, PUMP_ID);
    strcpy(systemInfo.ipAddr, _ipAddr.get_ip_address());
    strcpy(systemInfo.macAddr, _eth.get_mac_address());
    
    _socket->send((char*) &systemInfo, sizeof(SystemInfo));
}

/*! LED Functions */
void EZOSensors::flipStatusLED() {
    _redLED = !_redLED;
    _statusLED = !_statusLED;
}

/*! Initialising Ethernet */
void EZOSensors::initEthernet() {
    // Set static IP
    _eth.set_network(IP_ADDRESS, NETW_MASK, GATEWAY);

    // Bring up the ethernet interface
    _eth.connect();

    // Show the network address
    _eth.get_ip_address(&_ipAddr);

    // Open a socket on the network interface, and create a TCP connection to mbed.org
    _server.open(&_eth);
    _server.bind(7851);
    _server.listen(1);
    _server.set_blocking(true);
    _server.set_timeout(-1);
}

/*! Getting a function pointer based on the FID */
const EZOSensors::ComMessage* EZOSensors::getComFromHeader(const MessageHeader* header) {

    if (header->fid >= _fidCount) { //Prevent getting out of an array
        return NULL;
    }

    return &comMessages[header->fid];
}

void EZOSensors::comReturn(const void* data, const int errorCode) {
    MessageHeader *message = (MessageHeader*) data;
    message->packetLength = _msgHeaderLength;
    message->error = errorCode;
    _socket->send((char*) message, _msgHeaderLength);
}

/*! Setting the pump state */
void EZOSensors::setBoardState(int state) {
    
    _boardState = state;
    
    switch(_boardState) {
        case WAIT_FOR_CONNECTION:
            // Blink status LED
            _tickerEZO.attach(callback(this, &EZOSensors::flipStatusLED), 500ms);
            break;
        case CONNECTED:
            // Solid status LED
            _tickerEZO.detach();
            _redLED = 0;
            _statusLED = 1;
            break;
        default:
            _tickerEZO.detach();
            _redLED = 0;
            _statusLED = 1;
            break;
    }
}

/*! Main function */
void EZOSensors::run() {
    // Indicate initialising state of a system
    setBoardState(WAIT_FOR_CONNECTION);
    
    initEthernet();

    // Initialise data buffer (receive)
    char data[256];
    // Create pointer to header data
    MessageHeader* header;

    while (true) {
        // Indicate state of a system
        setBoardState(WAIT_FOR_CONNECTION);
        
        _socket = _server.accept();
        _socket->getpeername(&_clientAddr);
        // _socket->set_timeout(1000);
        
        // Indicate state of a system
        setBoardState(CONNECTED);
        
        while(true) {
            // Wait for a header
            _socketBytes = _socket->recv(data, _msgHeaderLength);
            if (_socketBytes <= 0) break;
            
            // if (_socketBytes > 0) {
            header = (MessageHeader*)data;
            
            if (header->packetLength != _msgHeaderLength) {
                if (_socket->recv(data + _msgHeaderLength, header->packetLength - _msgHeaderLength) <= 0) break;
            }
            
            const ComMessage* comMessage = getComFromHeader(header);
            
            if(comMessage != NULL && comMessage->replyFunc != NULL) {
                // Allow only pump stop and status commands when pump is running
                // Fact: comMessage->fid is equivalent to (*comMessage).fid
                (this->*comMessage->replyFunc)((void*)data);
            } else {
                comReturn(data, MSG_ERROR_NOT_SUPPORTED);
            }
            // } else if (_socketBytes = -3001) { // recv has timed out
            //     _tempReading = _tempSensor.receiveReading();
            //     if (_tempSetpoint < _tempReading) {
            //         _pwmDutyCycle = 0;
            //     } else {
            //         _pwmDutyCycle = 1;
            //     }
            //     _tempSensor.sendReadCMD();
            // } else break;
        }
        
        // Client disconnected
        _socket->close();
        
        // Indicate disconnected state
        setBoardState(WAIT_FOR_CONNECTION);

        _readingPending = false;
        _initSetpoint = true;
        _CMDRead = false;
        _tempSetpoint = 0;
        _pid_1.reset();
        _pid_2.reset();
        _pid_3.reset();
        // _pwmDutyCycle.write(0);
        _phSensor.sleep();
        _tempSensor_1.sleep();
        _tempSensor_2.sleep();
        _tempSensor_3.sleep();
    }
}

