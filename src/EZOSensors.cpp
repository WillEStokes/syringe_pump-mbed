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
};

/*! Parameterised constructor */
EZOSensors::EZOSensors(
        PinName redLED,
        PinName statusLED)
        : _redLED(redLED),
        _statusLED(statusLED),
        _fidCount(sizeof (comMessages) / sizeof (ComMessage)), // constant
        _msgHeaderLength(sizeof (MessageHeader)) { //
    // Turn LED ON by default
    _redLED = 0;
    _statusLED = 1;
    
    //initialise non-constant variables
    _phSensorConnected = false;
    _tempSensorConnected = false;
    _CMDRead = false;
    _readingPending = false;
}

EZO phSensor(I2C_SDA, I2C_SCL, PH_ADDRESS); // sda, scl, address
EZO tempSensor(I2C_SDA, I2C_SCL, TEMP_ADDRESS); // sda, scl, address

/*! Get status */
void EZOSensors::getStatus(const MessageHeader* data) { 
    static SystemStatus status; // static is needed to avoid memory allocation every time the function is called
    
    status.header.packetLength = sizeof(SystemStatus);
    status.header.fid = FID_GET_STATUS;
    
    checkSensorConnection();
    
    if (!_phSensorConnected || !_tempSensorConnected) {
        status.header.error = MSG_ERROR_SENSOR_DISCONNECTED;
    } else {
        status.header.error = MSG_OK;
    }
    
    status.boardState = _boardState;
    status.phSensorConnected = _phSensorConnected;
    status.tempSensorConnected = _tempSensorConnected;
    
    _socket->send((char*) &status, sizeof(SystemStatus));
}

/*! Send read cmd */
void EZOSensors::sendReadCMD(const MessageHeader* data) {
    if (_readingPending) {
        return;
    }
    
    phSensor.sendReadCMD();
    tempSensor.sendReadCMD();
    
    _tickerReadCMD.attach(callback(this, &EZOSensors::receiveReading), 1s);
    _readingPending = true;
}

void EZOSensors::receiveReading() {
    _tickerReadCMD.detach();
    
    _readingPending = false;
    _CMDRead = true;
}

/*! Get sensor data */
void EZOSensors::getSensorData(const MessageHeader* data) {
    static SensorData sensorData;

    if (!_readingPending) {
        _phReading = phSensor.receiveReading();
        _tempReading = tempSensor.receiveReading();
    }
    
    sensorData.header.packetLength = sizeof(SensorData);
    sensorData.header.fid = FID_GET_SENSOR_DATA;
    sensorData.ph = _phReading;
    sensorData.temp = _tempReading;
    
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
    
    _phSensorInfo = phSensor.getSensorInfo();
    _tempSensorInfo = tempSensor.getSensorInfo();
    
    strcpy(sensorInfo.ph, _phSensorInfo.c_str());
    strcpy(sensorInfo.temp, _tempSensorInfo.c_str());
    
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
    
    _phSensorStatus = phSensor.getSensorStatus();
    _tempSensorStatus = tempSensor.getSensorStatus();
    
    strcpy(sensorStatus.ph, _phSensorStatus.c_str());
    strcpy(sensorStatus.temp, _tempSensorStatus.c_str());
    
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
        
        if (_tempReading != -1023) {
            _tempSensorConnected = true;
        } else {
            _tempSensorConnected = false;
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
            _tickerStatus.attach(callback(this, &EZOSensors::flipStatusLED), 500ms);
            break;
        case CONNECTED:
            // Solid status LED
            _tickerStatus.detach();
            _redLED = 0;
            _statusLED = 1;
            break;
        default:
            _tickerStatus.detach();
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
        
        // Indicate state of a system
        setBoardState(CONNECTED);
        
        while(true) {
            // Wait for a header
            if (_socket->recv(data, _msgHeaderLength) <= 0) break;
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
        }
        
        // Client disconnected
        _socket->close();
        
        // Indicate disconnected state
        setBoardState(WAIT_FOR_CONNECTION);

        phSensor.sleep();
        tempSensor.sleep();
    }
}

