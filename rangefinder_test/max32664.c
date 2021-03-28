#include "SparkFun_Bio_Sensor_Hub_Library.h"


//---------------------------------------------------------
// Variables
//---------------------------------------------------------

uint8_t bpmArr[MAXFAST_ARRAY_SIZE];
uint8_t bpmArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA];
uint8_t senArr[MAX30101_LED_ARRAY];
uint8_t bpmSenArr[MAXFAST_ARRAY_SIZE + MAX30101_LED_ARRAY];
uint8_t bpmSenArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA + MAX30101_LED_ARRAY] ;
uint8_t _resetPin;
uint8_t _mfioPin;
uint8_t _address=0x55;
uint32_t _writeCoefArr[3];
uint8_t _userSelectedMode;
uint8_t _sampleRate = 100;

//---------------------------------------------------------
// Private Function
//---------------------------------------------------------

uint8_t readSensorHubStatus();
uint8_t enableWrite(uint8_t _familyByte, uint8_t _indexByte,\
                    uint8_t _enableByte);
uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte,\
                  uint8_t _writeByte);
uint8_t readByte(uint8_t _familyByte, uint8_t _indexByte );
uint8_t readByte_alter(uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);
uint8_t setOutputMode(uint8_t outputType);
uint8_t setFifoThreshold(uint8_t intThresh);
uint8_t agcAlgoControl(uint8_t enable);
uint8_t max30101Control(uint8_t senSwitch);
uint8_t maximFastAlgoControl(uint8_t mode);
uint8_t readAlgoSamples();
uint8_t readFillArray(uint8_t _familyByte, uint8_t _indexByte,\
                      uint8_t _numOfReads, uint8_t array[] );


//---------------------------------------------------------
// Pulse sensor hub Function
//---------------------------------------------------------


uint8_t max32664_begin(uint16_t, uint16_t) {

    
    _resetPin = resetPin;
    _mfioPin = mfioPin;
    DDRC |= (1<<_mfioPin);
    DDRC |= (1<<_resetPin);
      // Set these pins as output


  PORTC |= (1<<_mfioPin);//digitalWrite(_mfioPin, HIGH);
  PORTC &= ~(1<<_resetPin);//digitalWrite(_resetPin, LOW);
  _delay_ms(10);
  PORTC |= (1<<_resetPin);//digitalWrite(_resetPin, HIGH);
  _delay_ms(1000);
  PORTC |= (1<<_mfioPin);//pinMode(_mfioPin, INPUT_PULLUP); // To be used as an interrupt later

  uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte.
  return responseByte;
}


uint8_t max32664_configBpm(uint8_t mode){

  uint8_t statusChauf = 0;
  if (mode == MODE_ONE || mode == MODE_TWO){}
  else return INCORR_PARAM;

  statusChauf = setOutputMode(ALGO_DATA); // Just the data
  if( statusChauf != SUCCESS )
    return statusChauf;

  statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired.
  if( statusChauf != SUCCESS )
    return statusChauf;

  statusChauf = agcAlgoControl(ENABLE); // One sample before interrupt is fired.
  if( statusChauf != SUCCESS )
    return statusChauf;

  statusChauf = max30101Control(ENABLE);
  if( statusChauf != SUCCESS )
    return statusChauf;

  statusChauf = maximFastAlgoControl(mode);
  if( statusChauf != SUCCESS )
    return statusChauf;

  _userSelectedMode = mode;
  _sampleRate = readAlgoSamples();

  _delay_ms(1000);
  return SUCCESS;

}

bioData max32664_readBpm(){

  bioData libBpm;
  uint8_t statusChauf; // The status chauffeur captures return values.

  statusChauf = readSensorHubStatus();

  if (statusChauf == 1){ // Communication Error
    libBpm.heartRate = 0;
    libBpm.confidence = 0;
    libBpm.oxygen = 0;
    return libBpm;
  }

  numSamplesOutFifo();

  if (_userSelectedMode == MODE_ONE) {

    readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpmArr);

    // Heart Rate formatting
    libBpm.heartRate = (uint16_t(bpmArr[0]) << 8);
    libBpm.heartRate |= (bpmArr[1]);
    libBpm.heartRate /= 10;

    // Confidence formatting
    libBpm.confidence = bpmArr[2];

    //Blood oxygen level formatting
    libBpm.oxygen = uint16_t(bpmArr[3]) << 8;
    libBpm.oxygen |= bpmArr[4];
    libBpm.oxygen /= 10;

    //"Machine State" - has a finger been detected?
    libBpm.status = bpmArr[5];

    return libBpm;
  }

  else if (_userSelectedMode == MODE_TWO) {
    readFillArray(READ_DATA_OUTPUT, READ_DATA,\
        MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);

    // Heart Rate formatting
    libBpm.heartRate = (uint16_t(bpmArrTwo[0]) << 8);
    libBpm.heartRate |= (bpmArrTwo[1]);
    libBpm.heartRate /= 10;

    // Confidence formatting
    libBpm.confidence = bpmArrTwo[2];

    //Blood oxygen level formatting
    libBpm.oxygen = uint16_t(bpmArrTwo[3]) << 8;
    libBpm.oxygen |= bpmArrTwo[4];
    libBpm.oxygen /= 10.0;

    //"Machine State" - has a finger been detected?
    libBpm.status = bpmArrTwo[5];

    //Sp02 r Value formatting
    uint16_t tempVal = uint16_t(bpmArrTwo[6]) << 8;
    tempVal |= bpmArrTwo[7];
    libBpm.rValue = tempVal;
    libBpm.rValue /= 10.0;

    //Extended Machine State formatting
    libBpm.extStatus = bpmArrTwo[8];

    // There are two additional bytes of data that were requested but that
    // have not been implemented in firmware 10.1 so will not be saved to
    // user's data.
    return libBpm;
  }

  else {
    libBpm.heartRate  = 0;
    libBpm.confidence = 0;
    libBpm.oxygen     = 0;
    return libBpm;
  }

}

//---------------------------------------------------------
// Private Function
//---------------------------------------------------------


uint8_t readSensorHubStatus(){

  uint8_t status = readByte(0x00, 0x00); // Just family and index byte.
  return status; // Will return 0x00

}

uint8_t enableWrite(uint8_t _familyByte, uint8_t _indexByte,\
                                                                uint8_t _enableByte)
{
    i2c_start(_address | I2C_WRITE );
    i2c_write(_familyByte);
    i2c_write(_indexByte);
    i2c_write(_enableByte);
    _delay_ms(CMD_DELAY);
    i2c_rep_start( _address | I2C_READ );
    uint8_t statusByte = i2c_readNak();
    
    i2c_stop();
  return statusByte;

}

uint8_t writeByte(uint8_t _familyByte, uint8_t _indexByte,\
                                                                uint8_t _writeByte)
{

    i2c_start(_address | I2C_WRITE );
    i2c_write(_familyByte);
    i2c_write(_indexByte);
    i2c_write(_writeByte);
    _delay_ms(CMD_DELAY);
    i2c_rep_start( _address | I2C_READ );
    uint8_t statusByte = i2c_readNak();
    
    i2c_stop();

    return statusByte;

}

uint8_t readByte(uint8_t _familyByte, uint8_t _indexByte )
{

  uint8_t returnByte;
  uint8_t statusByte;
    
    i2c_start(_address | I2C_WRITE );
       i2c_write(_familyByte);
       i2c_write(_indexByte);
       _delay_ms(CMD_DELAY);
    i2c_rep_start( _address | I2C_READ );
    statusByte = i2c_readAck();

    if( statusByte ){// SUCCESS (0x00)
        return statusByte;} // Return the error, see: READ_STATUS_BYTE_VALUE

       returnByte = = i2c_readNak();
       return returnByte; // If good then return the actual byte.

}

uint8_t  readByte_alter(uint8_t _familyByte, uint8_t _indexByte,\
                                           uint8_t _writeByte)
{

  uint8_t returnByte;
  uint8_t statusByte;

    i2c_start(_address | I2C_WRITE );
    i2c_write(_familyByte);
    i2c_write(_indexByte);
    i2c_write(_writeByte);

    _delay_ms(CMD_DELAY);
    i2c_rep_start( _address | I2C_READ );
    statusByte = i2c_readAck();

    if( statusByte ){// SUCCESS (0x00)
        return statusByte;} // Return the error, see: READ_STATUS_BYTE_VALUE

  returnByte = = i2c_readNak();
  return returnByte; // If good then return the actual byte.

}

uint8_t setOutputMode(uint8_t outputType) {

  if ( outputType > SENSOR_ALGO_COUNTER ) // Bytes between 0x00 and 0x07
    return INCORR_PARAM;

  // Check that communication was successful, not that the IC is outputting
  // correct format.
  uint8_t statusByte = writeByte(OUTPUT_MODE, SET_FORMAT, outputType);
  if( statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;

}

uint8_t setFifoThreshold(uint8_t intThresh) {

  // Checks that there was succesful communcation, not that the threshold was
  // set correctly.
  uint8_t statusByte = writeByte(OUTPUT_MODE, WRITE_SET_THRESHOLD, intThresh);
  if( statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;

}

uint8_t agcAlgoControl(uint8_t enable) {

  if( enable == 0 || enable == 1) {}
  else
    return INCORR_PARAM;

  uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_AGC_ALGO, enable);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;

}

uint8_t max30101Control(uint8_t senSwitch) {

  if(senSwitch == 0 || senSwitch == 1)
    { }
  else
    return INCORR_PARAM;

  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = enableWrite(ENABLE_SENSOR, ENABLE_MAX30101, senSwitch);
  if( statusByte != SUCCESS )
    return statusByte;
  else
    return SUCCESS;

}

uint8_t maximFastAlgoControl(uint8_t mode) {

  if( mode == 0 || mode == 1 || mode == 2) {}
  else
    return INCORR_PARAM;

  uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;

}

uint8_t readAlgoSamples() {

  uint8_t samples = readByte_alter(READ_ALGORITHM_CONFIG, READ_AGC_NUM_SAMPLES, READ_AGC_NUM_SAMPLES_ID );
  return samples;

}

uint8_t readFillArray(uint8_t _familyByte, uint8_t _indexByte,\
                                                uint8_t _numOfReads, uint8_t array[] )
{

  uint8_t statusByte;
    
    i2c_start(_address | I2C_WRITE );
    i2c_write(_familyByte);
    i2c_write(_indexByte);
    _delay_ms(CMD_DELAY);
    i2c_rep_start( _address | I2C_READ );
    uint8_t statusByte = i2c_readAck();
    

    while ( _numOfReads > 0 ) {
        if ( _numOfReads > 1 ){
            array[_numOfReads] = i2c_readAck();
            } else {
            array[_numOfReads] = i2c_readNak();
        }
        _numOfReads--;
    }
  i2c_stop();
  return statusByte;

}

