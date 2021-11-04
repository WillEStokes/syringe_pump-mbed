#include "mbed.h"
#include "EZO.h"

EZO::EZO(PinName sda, PinName scl, char slave_adr)
    :
    i2c_p(new I2C(sda, scl)),
    i2c(*i2c_p),
    address(slave_adr)
{
    clear();
    i2c.frequency(100000);
    error255 = "No Data";
    error254 = "Pending";
    failedStr = "Failed";
    succesStr = "Success";
    failed = 2;
    success = 1;
}

EZO::~EZO()
{
    if (NULL != i2c_p)
        delete  i2c_p;
}

void EZO::clear(void)
{
    for (int i=0; i<sizeof(ezodata); i++) {
        ezodata[i] = '0';
    }
    for (int i=0; i<sizeof(cmdData); i++) {
        cmdData[i] = '0';
    }
    for (int i=0; i<sizeof(receivedValue); i++) {
        receivedValue[i] = '0';
    }
    ezovalue = "0";
}

int EZO::Tcompensation (float tmp)
{
    clear();
    int value = static_cast<int>(tmp);
    char dig[3] = { NULL, NULL, NULL};
    int x = 0;
    while (value > 0 && x < 3) {
        dig[x] = value % 10;
        value /= 10;
        x++;
    }

    cmdData[0] = 'T';
    cmdData[1] = ',';
    cmdData[2] = dig[0];
    cmdData[3] = '.';
    cmdData[4] = dig[1];
    cmdData[5] = dig[2];

    i2c.write(address, cmdData, 6, false);
    // ThisThread::sleep_for(chrono::milliseconds(300));
    thread_sleep_for(300);

    i2c.read(address, ezodata, 1, false);

    if( ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

float EZO::QTcompensation (void)
{
    clear();
    cmdData[0] = 'T';
    cmdData[1] = ',';
    cmdData[3] = '?';

    i2c.write(address, cmdData, 3, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 10, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[4];
        receivedValue[1] = ezodata[5];
        receivedValue[2] = ezodata[6];
        receivedValue[3] = ezodata[7];
        float tmp = atof(receivedValue);
        return tmp;
    } else {
        return 0.0;
    }
}

string EZO::getSensorInfo(void)
{
    clear();
    cmdData[0] = 'I';

    i2c.write(address, cmdData, 1, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        return receivedValue;
    } else if(ezodata[0] == 254) {
        return error254;
    } else if(ezodata[0] == 255) {
        return error255;
    } else {
        return failedStr;
    }
}

string EZO::getSensorStatus(void)
{
    clear();
    cmdData[0] = 'S';
    cmdData[1] = 'T';
    cmdData[2] = 'A';
    cmdData[3] = 'T';
    cmdData[4] = 'U';
    cmdData[5] = 'S';

    i2c.write(address, cmdData, 6, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        return receivedValue;
    } else if(ezodata[0] == 254) {
        return error254;
    } else if(ezodata[0] == 255) {
        return error255;
    } else {
        return failedStr;
    }
}

float EZO::read()
{
    clear();
    cmdData[0] = 'R';

    i2c.write(address, cmdData, 1, false);
    thread_sleep_for(800);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        float sensorValue = atof(receivedValue);
        return sensorValue;
    } else {
        return -1;
    }
}

void EZO::sendReadCMD()
{
    clear();
    cmdData[0] = 'R';

    i2c.write(address, cmdData, 1, false);
}

float EZO::receiveReading()
{
    clear();
    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        float sensorValue = atof(receivedValue);
        return sensorValue;
    } else {
        return -1;
    }
}

int EZO::sensorLED(bool on)
{
    clear();
    cmdData[0] = 'L';
    cmdData[1] = ',';
    if(on) {
        cmdData[2] = '1';
    } else {
        cmdData[2] = '0';
    }
    i2c.write(address, cmdData, 3, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 1, false);
    if( ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

int EZO::QsensorLED(void)
{
    clear();
    cmdData[0] = 'L';
    cmdData[1] = ',';
    cmdData[2] = '?';

    i2c.write(address, cmdData, 3, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 6, false);

    if(ezodata[0] == 1) {
        if(ezodata[4] == '1') {
            return 1;
        } else if(ezodata[4] == '0') {
            return 0;
        }
    } else {
        return failed;
    }
}

int EZO::calibrationClear(void)
{
    clear();
    cmdData[0] = 'C';
    cmdData[1] = 'a';
    cmdData[2] = 'l';
    cmdData[3] = ',';
    cmdData[4] = 'c';
    cmdData[5] = 'l';
    cmdData[6] = 'e';
    cmdData[7] = 'a';
    cmdData[8] = 'r';

    i2c.write(address, cmdData, 9, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 1, false);

    if(ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

string EZO::calibrationQuery(void)
{
    clear();
    cmdData[0] = 'C';
    cmdData[1] = 'a';
    cmdData[2] = 'l';
    cmdData[3] = ',';
    cmdData[4] = '?';

    i2c.write(address, cmdData, 5, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        return receivedValue;
    } else if(ezodata[0] == 254) {
        return error254;
    } else if(ezodata[0] == 255) {
        return error255;
    } else {
        return failedStr;
    }
}

int EZO::calibratingMid(void)
{
    clear();
    cmdData[0] = 'C';
    cmdData[1] = 'a';
    cmdData[2] = 'l';
    cmdData[3] = ',';
    cmdData[4] = 'm';
    cmdData[5] = 'i';
    cmdData[6] = 'd';
    cmdData[7] = ',';
    cmdData[8] = '7';
    cmdData[9] = '.';
    cmdData[10] = '0';
    cmdData[11] = '0';

    i2c.write(address, cmdData, 12, false);
    thread_sleep_for(1600);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

int EZO::calibratingLow(void)
{
    clear();
    cmdData[0] = 'C';
    cmdData[1] = 'a';
    cmdData[2] = 'l';
    cmdData[3] = ',';
    cmdData[4] = 'l';
    cmdData[5] = 'o';
    cmdData[6] = 'w';
    cmdData[7] = ',';
    cmdData[8] = '4';
    cmdData[9] = '.';
    cmdData[10] = '0';
    cmdData[11] = '0';

    i2c.write(address, cmdData, 12, false);
    thread_sleep_for(1600);

    i2c.read(address, ezodata, 1, false);

    if(ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

int EZO::calibratingHigh(void)
{
    clear();
    cmdData[0] = 'C';
    cmdData[1] = 'a';
    cmdData[2] = 'l';
    cmdData[3] = ',';
    cmdData[4] = 'h';
    cmdData[5] = 'i';
    cmdData[6] = 'g';
    cmdData[7] = 'h';
    cmdData[8] = ',';
    cmdData[9] = '1';
    cmdData[10] = '0';
    cmdData[11] = '.';
    cmdData[12] = '0';
    cmdData[13] = '0';

    i2c.write(address, cmdData, 14, false);
    thread_sleep_for(1600);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

string EZO::slope(void)
{
    clear();
    cmdData[0] = 'S';
    cmdData[1] = 'l';
    cmdData[2] = 'o';
    cmdData[3] = 'p';
    cmdData[4] = 'e';
    cmdData[5] = ',';
    cmdData[6] = '?';

    i2c.write(address, cmdData, 7, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        return receivedValue;
    } else if(ezodata[0] == 254) {
        return error254;
    } else if(ezodata[0] == 255) {
        return error255;
    } else {
        return failedStr;
    }
}

string EZO::factoryReset(void)
{
    clear();
    cmdData[0] = 'F';
    cmdData[1] = 'a';
    cmdData[2] = 'c';
    cmdData[3] = 't';
    cmdData[4] = 'o';
    cmdData[5] = 'r';
    cmdData[6] = 'y';

    i2c.write(address, cmdData, 7, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 21, false);

    if(ezodata[0] == 1) {
        receivedValue[0] = ezodata[1];
        receivedValue[1] = ezodata[2];
        receivedValue[2] = ezodata[3];
        receivedValue[3] = ezodata[4];
        receivedValue[4] = ezodata[5];
        receivedValue[5] = ezodata[6];
        receivedValue[6] = ezodata[7];
        receivedValue[7] = ezodata[8];
        receivedValue[8] = ezodata[9];
        receivedValue[9] = ezodata[10];
        receivedValue[10] = ezodata[11];
        receivedValue[11] = ezodata[12];
        receivedValue[12] = ezodata[13];
        receivedValue[13] = ezodata[14];
        receivedValue[14] = ezodata[15];
        receivedValue[15] = ezodata[16];
        receivedValue[16] = ezodata[17];
        receivedValue[17] = ezodata[18];
        receivedValue[18] = ezodata[19];
        receivedValue[19] = ezodata[20];
        return receivedValue;
    } else if(ezodata[0] == 254) {
        return error254;
    } else if(ezodata[0] == 255) {
        return error255;
    } else {
        return failedStr;
    }
}

void EZO::sleep(void)
{
    clear();

    cmdData[0] = 'S';
    cmdData[1] = 'l';
    cmdData[2] = 'e';
    cmdData[3] = 'e';
    cmdData[4] = 'p';

    i2c.write(address, cmdData, 5, false);
    thread_sleep_for(300);

    // no response code
}

int EZO::EDProtocolLock(bool on)
{
    clear();
    cmdData[0] = 'P';
    cmdData[1] = 'L';
    cmdData[2] = 'O';
    cmdData[3] = 'C';
    cmdData[4] = 'K';
    cmdData[5] = ',';
    if(on) {
        cmdData[6] = '1';
    } else {
        cmdData[6] = '0';
    }

    i2c.write(address, cmdData, 7, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 1, false);

    if(ezodata[0] == 1) {
        return success;
    } else {
        return failed;
    }
}

int EZO::QProtocolLock()
{
    clear();
    cmdData[0] = 'P';
    cmdData[1] = 'L';
    cmdData[2] = 'O';
    cmdData[3] = 'C';
    cmdData[4] = 'K';
    cmdData[5] = ',';
    cmdData[6] = '?';

    i2c.write(address, cmdData, 7, false);
    thread_sleep_for(300);

    i2c.read(address, ezodata, 1, false);

    if(ezodata[0] == 1) {
        return ezodata[8];
    } else {
        return failed;
    }
}

void EZO::changeUART(int baudRate)
{
    clear();

    char dig[6] = { NULL, NULL, NULL, NULL, NULL, NULL};
    int x = 0;
    int value = baudRate;
    int number = 10;

    while (value > 0 && x < 3) {
        dig[x] = value % 10;
        value /= 10;
        x++;
    }
    cmdData[0] = 'S';
    cmdData[1] = 'E';
    cmdData[2] = 'R';
    cmdData[3] = 'I';
    cmdData[4] = 'A';
    cmdData[5] = 'L';
    cmdData[6] = ',';
    cmdData[7] = dig[0];
    cmdData[8] = dig[1];
    cmdData[9] = dig[2];
    if (dig[3] != NULL) {
        cmdData[10] = dig[3];
        number = 11;
    } else if (dig[4] != NULL) {
        cmdData[11] = dig[3];
        number = 12;
    } else if (dig[5] != NULL) {
        cmdData[12] = dig[3];
        number = 13;
    }

    i2c.write(address, cmdData, number, false);
    thread_sleep_for(300);
    // No response code
}
