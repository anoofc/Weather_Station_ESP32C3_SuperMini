/***************************************************************************************************/

/*
This is an Arduino library for Aosong ASAIR AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B

Digital Humidity & Temperature Sensor

written by : enjoyneering
source code: https://github.com/enjoyneering/

<...license, feature list omitted for brevity...>
*/

/***************************************************************************************************/

#include "AHTxx.h"

/**************************************************************************/

/*
Constructor
*/

/**************************************************************************/

AHTxx::AHTxx(uint8_t address, AHTXX_I2C_SENSOR sensorType, TwoWire* wire)
{
    _address = address;
    _sensorType = sensorType;
    _status = AHTXX_NO_ERROR;
    _wire = wire ? wire : &Wire;
}

/**************************************************************************/

/*
begin()
Initialize I2C & sensor

NOTE:
- call this function before doing anything else!!!
- speed in Hz, stretch in usec
- returned value by "Wire.endTransmission()":
  - 0 success
  - 1 data too long to fit in transmit data buffer
  - 2 received NACK on transmit of address
  - 3 received NACK on transmit of data
  - 4 other error
*/

/**************************************************************************/

#if defined (ARDUINO_ARCH_AVR)
bool AHTxx::begin(uint32_t speed, uint32_t stretch)
{
    _wire->begin();
    _wire->setClock(speed); //experimental! AVR I2C bus speed 31kHz..400kHz, default 100000Hz
    #if !defined (__AVR_ATtiny85__) //for backwards compatibility with ATtiny Core
    _wire->setWireTimeout(stretch, false); //experimental! default 25000usec, true=Wire hardware will be automatically reset to default on timeout
    #endif
#elif defined (ARDUINO_ARCH_ESP8266)
bool AHTxx::begin(uint8_t sda, uint8_t scl, uint32_t speed, uint32_t stretch)
{
    _wire->begin(sda, scl);
    _wire->setClock(speed); //experimental! ESP8266 I2C bus speed 1kHz..400kHz, default 100000Hz
    _wire->setClockStretchLimit(stretch); //experimental! default 150000usec
#elif defined (ARDUINO_ARCH_ESP32)
bool AHTxx::begin(int32_t sda, int32_t scl, uint32_t speed, uint32_t stretch) //"int32_t" for Master SDA & SCL, "uint8_t" for Slave SDA & SCL
{
    if (_wire->begin(sda, scl, speed) != true) {return false;} //ESP32 I2C bus speed ???kHz..400kHz, default 100000Hz
    _wire->setTimeout(stretch / 1000); //default 50msec
#elif defined (ARDUINO_ARCH_STM32)
bool AHTxx::begin(uint32_t sda, uint32_t scl, uint32_t speed) //"uint32_t" for pins only, "uint8_t" calls wrong "setSCL(PinName scl)"
{
    _wire->begin(sda, scl);
    _wire->setClock(speed); //STM32 I2C bus speed ???kHz..400kHz, default 100000Hz
#elif defined (ARDUINO_ARCH_SAMD)
bool AHTxx::begin(uint8_t columns, uint8_t rows, lcdFontSize fontSize, uint32_t speed)
{
    _wire->begin();
    _wire->setClock(speed); //SAMD21 I2C bus speed ???kHz..400kHz, default 100000Hz
#else
bool AHTxx::begin()
{
    _wire->begin();
#endif
    delay(AHT2X_POWER_ON_DELAY); //wait for sensor to initialize
    return softReset(); //soft reset is recommended at start (reset, set normal mode, set calibration bit & check calibration bit)
}

/**************************************************************************/

float AHTxx::readHumidity(bool readAHT)
{
    if (readAHT == AHTXX_FORCE_READ_DATA) {_readMeasurement();} //force to read data via I2C & update "_rawData[]" buffer
    if (_status != AHTXX_NO_ERROR) {return AHTXX_ERROR;} //no reason to continue, call "getStatus()" for error description

    uint32_t humidity = _rawData[1]; //20-bit raw humidity data
    humidity <<= 8;
    humidity |= _rawData[2];
    humidity <<= 4;
    humidity |= _rawData[3] >> 4;

    if (humidity > 0x100000) {humidity = 0x100000;} //check if RH>100; no need to check for RH<0 since "humidity" is "uint"
    return ((float)humidity / 0x100000) * 100;
}

/**************************************************************************/

float AHTxx::readTemperature(bool readAHT)
{
    if (readAHT == AHTXX_FORCE_READ_DATA) {_readMeasurement();} //force to read data via I2C & update "_rawData[]" buffer
    if (_status != AHTXX_NO_ERROR) {return AHTXX_ERROR;} //no reason to continue, call "getStatus()" for error description

    uint32_t temperature = _rawData[3] & 0x0F; //20-bit raw temperature data
    temperature <<= 8;
    temperature |= _rawData[4];
    temperature <<= 8;
    temperature |= _rawData[5];

    return ((float)temperature / 0x100000) * 200 - 50;
}

/**************************************************************************/

bool AHTxx::setNormalMode()
{
    return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE);
}

/**************************************************************************/

bool AHTxx::setCycleMode()
{
    return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_CYCLE_MODE);
}

/**************************************************************************/

bool AHTxx::setComandMode()
{
    return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_CMD_MODE);
}

/**************************************************************************/

bool AHTxx::softReset()
{
    _wire->beginTransmission(_address);
    _wire->write(AHTXX_SOFT_RESET_REG);
    if (_wire->endTransmission(true) != 0) {return false;} //collision on I2C bus, sensor didn't return ACK
    delay(AHTXX_SOFT_RESET_DELAY);
    return ((setNormalMode() == true) && (_getCalibration() == AHTXX_STATUS_CTRL_CAL_ON)); //set mode & check calibration bit
}

/**************************************************************************/

uint8_t AHTxx::getStatus()
{
    return _status;
}

/**************************************************************************/

void AHTxx::setType(AHTXX_I2C_SENSOR sensorType)
{
    _sensorType = sensorType;
}

/**************************************************************************/

bool AHTxx::_setInitializationRegister(uint8_t value)
{
    delay(AHTXX_CMD_DELAY);
    _wire->beginTransmission(_address);
    if (_sensorType == AHT1x_SENSOR) {_wire->write(AHT1X_INIT_REG);} //send initialization command, for AHT1x only
    else {_wire->write(AHT2X_INIT_REG);} //send initialization command, for AHT2x only
    _wire->write(value); //send initialization register controls
    _wire->write(AHTXX_INIT_CTRL_NOP); //send initialization register NOP control
    return (_wire->endTransmission(true) == 0); //true=success, false=I2C error
}

/**************************************************************************/

uint8_t AHTxx::_readStatusRegister()
{
    delay(AHTXX_CMD_DELAY);
    _wire->beginTransmission(_address);
    _wire->write(AHTXX_STATUS_REG);
    if (_wire->endTransmission(true) != 0) {return AHTXX_ERROR;} //collision on I2C bus, sensor didn't return ACK
    _wire->requestFrom(_address, (uint8_t)1, (uint8_t)true); //read 1-byte to rxBuffer, true-send stop after transmission
    if (_wire->available() == 1) {return _wire->read();} //read 1-byte from rxBuffer
    return AHTXX_ERROR; //collision on I2C bus, rxBuffer is empty
}

/**************************************************************************/

uint8_t AHTxx::_getCalibration()
{
    uint8_t value = _readStatusRegister();
    if (value != AHTXX_ERROR) {return (value & AHTXX_STATUS_CTRL_CAL_ON);} //0x08=loaded, 0x00=not loaded
    return AHTXX_ERROR; //collision on I2C bus, sensor didn't return ACK
}

/**************************************************************************/

uint8_t AHTxx::_getBusy(bool readAHT)
{
    if (readAHT == AHTXX_FORCE_READ_DATA) //force to read data via I2C & update "_rawData[]" buffer
    {
        delay(AHTXX_CMD_DELAY);
        _wire->requestFrom(_address, (uint8_t)1, (uint8_t)true); //read 1-byte to rxBuffer, true-send stop after transmission
        if (_wire->available() != 1) {return AHTXX_DATA_ERROR;} //no reason to continue, "return" terminates the entire function & "break" just exits the loop
        _rawData[0] = _wire->read(); //read 1-byte from rxBuffer
    }

    if ((_rawData[0] & AHTXX_STATUS_CTRL_BUSY) == AHTXX_STATUS_CTRL_BUSY) {_status = AHTXX_BUSY_ERROR;} //0x80=busy, 0x00=measurement completed
    else {_status = AHTXX_NO_ERROR;}
    return _status;
}

/**************************************************************************/

bool AHTxx::_checkCRC8()
{
    if (_sensorType == AHT2x_SENSOR)
    {
        uint8_t crc = 0xFF; //initial value
        for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex ++) //6-bytes in data, {status, RH, RH, RH+T, T, T, CRC}
        {
            crc ^= _rawData[byteIndex];
            for(uint8_t bitIndex = 8; bitIndex > 0; --bitIndex) //8-bits in byte
            {
                if (crc & 0x80) {crc = (crc << 1) ^ 0x31;} //0x31=CRC seed/polynomial
                else {crc = (crc << 1);}
            }
        }
        return (crc == _rawData[6]);
    }
    return true;
}

/**************************************************************************/

void AHTxx::_readMeasurement()
{
    /* send measurement command */
    _wire->beginTransmission(_address);
    _wire->write(AHTXX_START_MEASUREMENT_REG); //send measurement command, start measurement...
    _wire->write(AHTXX_START_MEASUREMENT_CTRL); //send measurement control
    _wire->write(AHTXX_START_MEASUREMENT_CTRL_NOP); //send measurement NOP control
    if (_wire->endTransmission(true) != 0) //collision on I2C bus
    {
        _status = AHTXX_ACK_ERROR; //update status byte, sensor didn't return ACK
        return; //no reason to continue
    }

    /* check busy bit */
    _status = _getBusy(AHTXX_FORCE_READ_DATA); //update status byte, read status byte & check busy bit
    if (_status == AHTXX_BUSY_ERROR) {delay(AHTXX_MEASUREMENT_DELAY - AHTXX_CMD_DELAY);}
    else if (_status != AHTXX_NO_ERROR) {return;} //no reason to continue, received data smaller than expected

    /* read data from sensor */
    uint8_t dataSize;
    if (_sensorType == AHT1x_SENSOR) {dataSize = 6;} //{status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only
    else {dataSize = 7;}

    _wire->requestFrom(_address, dataSize, (uint8_t)true); //read n-bytes to rxBuffer, true-send stop after transmission
    if (_wire->available() != dataSize)
    {
        _status = AHTXX_DATA_ERROR; //update status byte, received data smaller than expected
        return; //no reason to continue
    }

    /* read n-bytes from rxBuffer */
    _wire->readBytes(_rawData, dataSize); //"readBytes()" from Stream Class
    /* check busy bit after measurement delay */
    _status = _getBusy(AHTXX_USE_READ_DATA); //update status byte, read status byte & check busy bit
    if (_status != AHTXX_NO_ERROR) {return;} //no reason to continue, sensor is busy
    /* check CRC8, for AHT2x only */
    if ((_sensorType == AHT2x_SENSOR) && (_checkCRC8() != true)) {_status = AHTXX_CRC8_ERROR;} //update status byte
}

