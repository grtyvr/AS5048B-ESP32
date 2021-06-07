#include <Arduino.h>

#include "AS5048.hpp"

//#define AS5048A_DEBUG

// SPI Register Map
const uint16_t AS5048_CMD_NOP = 0x0;      // no operation, dummy information.  Use this to get result of last command
const uint16_t AS5048_REG_ERR = 0x1;      // Error Register.  To clear the register, access it.
                                          // bit 0, framing error, bit 1 Command invalid, bit 2 Parity Error.
const uint16_t AS5048_PRM_CTL = 0x3;      // Programming control register.  Must enable before burning fuses.  Always verify after program
                                          // bit 0: program enable - bit 3: burn -- bit 6: verify
const uint16_t AS5048_OTP_0_HIGH = 0x16;  // Zero position high byte: bits 0..7  top 6 bits not used.
const uint16_t AS5048_OTP_0_LOW = 0x17;   // Zero position lower 6 Least Significant Bits: bits 0..5, Top 8 bits not used.
const uint16_t AS5048_REG_AGC = 0x3FFD;   // Diagnostic and Automatic Gain Control
                                          // Bits 0..7 AGC value 0=high, 255=low - Bit 8: OCF - Bit 9: COF - Bits 10..11 Comp Low..High
const uint16_t AS5048_REG_MAG = 0x3FFE;   // Magnitude after ATAN calculation bits 0..13
const uint16_t AS5048_REG_ANGLE = 0x3FFF; // Angle after ATAN calculation and zero position correction if used - bits 0..13

const uint16_t AS5048_READ_CMD = 0x4000;  // bit 15 = 1 for read operation.   

SPIClass * vspi = NULL;

class Angle{
  public:
    // constructor with no arguments means that we have to set the angle / tics later
    Angle(){
      _tics = 0;
      _radians = 0.0;
      _x = 0.0;
      _y=0.0;
    }
    // constructor that takes an unsigned 16 bit number and sets the radians as a positive rotation (CCW) from zero
    Angle(uint16_t tics){
      _tics = tics;
      _radians = _angleIncrement * _tics;
      _x = cos(_radians);
      _y = sin(_radians);
    }
    // constructor that takes a float representing the rotation in radians and set the tics as a positive rotation (CCW) from zero
    Angle(double radians){
      _radians = radians;
      // first normalize the value so that it's absolute value is in [0, 2*PI)
      // do that by counting the number of full rotations in the input value (multiples of 2PI) using the floor function
      if (_radians >= 2*PI){
        // subtract the number of whole clockwise rotations over 1
        _radians = _radians - floor(_radians/(2*PI))*2*PI;
      } else if (_radians < 0) { 
        // our value is a clockwise rotation from zero.
        // so in this case add the number of whole rotations clockwise
        _radians = _radians + floor(abs(_radians)/(2*PI))*2*PI;
        // then normalize this to a positive rotation counter clockwise
        _radians = 2*PI + _radians;
      }
      // now we have a positive angle that is in the interval[0,2*PI)
      // so digitize that value to the nearest angular increment 
      _tics = (uint16_t) round(_radians / _angleIncrement);
      _x = cos(_radians);
      _y = sin(_radians);
    }

    void setTics(uint16_t tics){
      _tics = tics;
      _radians = _angleIncrement * _tics;
      _x = cos(_radians);
      _y = sin(_radians);
    }
    void setRadians(double radians){
      _radians = radians;
      // first normalize the value so that it's absolute value is in [0, 2*PI)
      // do that by counting the number of full rotations in the input value (multiples of 2PI) using the floor function
      if (_radians >= 2*PI){
        // subtract the number of whole clockwise rotations over 1
        _radians = _radians - floor(_radians/(2*PI))*2*PI;
      } else if (_radians < 0) { 
        // our value is a clockwise rotation from zero.
        // so in this case add the number of whole rotations clockwise
        _radians = _radians + floor(abs(_radians)/(2*PI))*2*PI;
        // then normalize this to a positive rotation counter clockwise
        _radians = 2*PI + _radians;
      }
      // now we have a positive angle that is in the interval[0,2*PI)
      // so digitize that value to the nearest angular increment 
      _tics = (uint16_t) round(_radians / _angleIncrement);
      _x = cos(_radians);
      _y = sin(_radians);
    }
    uint16_t getTics(){
      return _tics;
    }
    double getRadians(){
      return _radians;
    }
    double x(){
      return _x;
    }
    double y(){
      return _y;
    }

  private:
    double _angleIncrement = (2* PI) / pow(2,14);  // The smallest angle we can represent with our encoder
    uint16_t _tics = 0;                           // The number of tics that best represents an angle in radians
    double _radians = 0.0;                         // The number of radians in tics based on our smalles angular increment
    double _x = 0.0;                               // x component of the angle on unit circle
    double _y = 0.0;                               // y component of the angle on unit circle
    uint8_t _nullZone = 0;
};

AS5048A::AS5048A(uint8_t arg_cs, uint8_t nullZone){
  _cs = arg_cs;
  _nullZone = nullZone;
  pinMode(_cs, OUTPUT);
  _errorFlag = false;
  _angle = 0.0;                           // initialize to zero degrees
}

void AS5048A::init(uint32_t clock, uint8_t bitOrder, uint8_t dataMode){
  // set up the SPI interface so that we can communicate with the chip
  // use mode 1, 
  settings = SPISettings(clock, bitOrder, dataMode);
  vspi = new SPIClass(VSPI);
  // Wake up the bus
  vspi->begin();
}

void AS5048A::close(){
  vspi->end();
}

bool AS5048A::error(){
  return _errorFlag;
}

uint16_t AS5048A::getMagnitude(){
  uint16_t rawData = read(AS5048_REG_MAG);
  // the bottom 14 bits are the magnitude
  return rawData &= 0x3FFF;
}

uint16_t AS5048A::getAngle(){
  uint16_t rawData = read(AS5048_REG_ANGLE);
  // the bottom 14 bits are the angle
  rawData &= 0x3FFF;
  Serial.print(" null zone: ");
  Serial.print(_nullZone);
  Serial.print(" ");
  Serial.print(" old angle: ");
  Serial.print(_angle);
  Serial.print(" ");
  if (abs(rawData - _angle) > _nullZone){
    _angle = rawData; 
    return rawData &= 0x3FFF;
  } else {
    return _angle;
  }
}

uint16_t AS5048A::getExpSmoothAngle(float smoothingFactor){
  /// use exponential smoothing to return the new reading
  /// since we might be crossing from 2^14 back to 0 we need to take care about normalizing our readings
  uint16_t newAngle = read(AS5048_REG_ANGLE);
  // the bottom 14 bits are the angle
   newAngle &= 0x3FFF;
  // make sure we have not wrapped around
  // we do that by making sure that our old value is not more than half
  // of the total range away from the new value
  // For example:
  //    1 ---> 16383 is just 3 tic
  //  We want to do something reasonable with that sort of thing
  //
  if (_angle - newAngle > 8192){
    // we have wrapped from High to LOW
    // so move the new value out past the high end
    // calculate what the smoothed value would be as if the range was wider
    // and if that new value would move us out of range, then return
    // the wrapped value
    newAngle += 16384;
    _angle = (_angle * (1 - smoothingFactor)) + (newAngle * smoothingFactor);
  } else if (newAngle - _angle > 8192){
    // here we have wrapped from Low to High
    // so move the new value to the low end ( may be negative but it still works)
    // calculate what the smoothed value would be as if the range was wider
    // and if that new value would move us out of range, then return
    // the wrapped value
    newAngle -= 16384;
    _angle = (_angle * (1 - smoothingFactor)) + (newAngle * smoothingFactor);
    // this could be negative.  If it is we have to wrap back to the top....
    if (_angle < 0){
      _angle += 16384;
    }
  } else {
    _angle = (_angle * (1 - smoothingFactor)) + (newAngle * smoothingFactor);
  }
  return  ((uint16_t) round(_angle)) % 16384;
}

uint16_t AS5048A::getMeanAngle(int numSamples){
  uint16_t retVal;
  float meanX = 0.0;
  float meanY = 0.0;
  Angle sample;
  /// take a number of samples and return the circular mean value
  /// since we might have a situation where we are sampling at the transition from
  /// 2^14 - n to m for small integer n and small integer m, have to be carefull to normalize our data
  for (int i=0; i < numSamples; i++){
    // take a sample and compute the x and y coordinate of the sample as if it on the unit circle
    sample.setTics(read(AS5048_REG_ANGLE));
    meanX += sample.x();
    meanY += sample.y();
  }
  // Take the average X and average Y values
  meanX = meanX/numSamples;
  meanY = meanY/numSamples;
  if ( (meanX == 0.0) & (meanY ==0.0) ){
    // pathalogical case of both X and Y being equal to zero as floats!
    retVal = 0;
  } else {
    Angle angle(atan2(meanY, meanX));
    if (abs(_angle - angle.getTics()) > _nullZone){
      retVal = angle.getTics();
    } else {
      retVal = _angle;
    }
    _angle = angle.getTics();
  }
  return retVal; 
}

void AS5048A::printDiagnostics(){
  uint16_t rawData = getDiag();
  Serial.print("AGC Value: ");
  // bottom 8 bits are the AGC value
  Serial.println(rawData & 0xFF);
  // bit 9 is OCF
  Serial.print("Offset Compensation Finished: ");
  Serial.print((rawData >> 8) & 0x1);
  // bit 10 is COF
  Serial.print(" - Cordic OverFlow: ");
  Serial.println((rawData >> 9) & 0x1);
    // bit 11 is Comp Low
  Serial.print("Comp Low: ");
  Serial.print((rawData >> 10) & 0x1);
  // bit 12 is Comp High
  Serial.print(" - Comp High: ");
  Serial.println((rawData >> 11) & 0x1);
}

uint16_t AS5048A::getDiag(){
  return read(AS5048_REG_AGC);
}

uint8_t AS5048A::getGain(){
  // the gain is in the bottom 8 bits
  return read(AS5048_REG_AGC) & 0xFF;
}

uint8_t AS5048A::getErrors(){
  _errorFlag = false;
  // To get the value of the error flags we have to send a special read command
  // The command clears the ERROR FLAG which is contained in every READ frame.
  // read the error register of the last command
  uint8_t retVal;
  // Set up the command we will send
  uint16_t cmdClearErrorFlag = 0x4001;  // 0b0100000000000001
  vspi->beginTransaction(settings);
  // Drop cs low to enable the AS5048
  digitalWrite(_cs, LOW);
  retVal = vspi->transfer16(cmdClearErrorFlag);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cs, LOW);
  // the next two commands are NOP commands 
  // the first one is to trigger the return of the Error Register contents
  // and will still have the Error Flag Set.
  retVal = vspi->transfer16(0x0);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cs, LOW);
  // The second one will trigger the clearing of the Error Flag and we do not
  // get any usefull information back with that command
  vspi->transfer16(0x0);
  digitalWrite(_cs, HIGH);
  vspi->endTransaction();
  #ifdef AS5048A_DEBUG
    Serial.print("Sent Command: ");
    Serial.println(command, HEX);
    Serial.print(" To register: ");
    Serial.println(registerAddress, BIN);
    Serial.print("Read returned: ");
    Serial.println(data);
  #endif

  // the bottom 3 bits store the error value
  // Bit 2: parity error
  // Bit 1: Invalid Command
  // Bit 0: Framing error
  return retVal &= 0x7;
}

uint16_t AS5048A::read(uint16_t registerAddress){
  uint16_t data;
  // Set up the command we will send
  uint16_t command = AS5048_READ_CMD | registerAddress;
  // leftmost bit is an even parity bit.
  command |= calcEvenParity(command) << 15;
  vspi->beginTransaction(settings);
  // Drop cs low to enable the AS5048
  digitalWrite(_cs, LOW);
  data = vspi->transfer16(command);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cs, LOW);
  // you have to poll the chip twice.  Data from previous command comes
  // back on the next SPI transfer so we send a dummy operation to get
  // the result of our last command
  data = vspi->transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  vspi->endTransaction();
  #ifdef AS5048A_DEBUG
    Serial.print("Sent Command: ");
    Serial.println(command, HEX);
    Serial.print(" To register: ");
    Serial.println(registerAddress, BIN);
    Serial.print("Read returned: ");
    Serial.println(data);
  #endif
  // on a data packet Bit 15 is an error flag so lets check for errors
  if(data & 0x4000){
    #ifdef AS5048A_DEBUG
      Serial.println("Setting error bit");
    #endif
    _errorFlag = true;
  } else {
    _errorFlag = false;
  }
  // value is in the bottom 14 bits
  return data &= 0x3FFF;
}

uint8_t AS5048A::calcEvenParity(uint16_t value){
  uint8_t count = 0;
  for (int i = 0; i < 16; i++) {
    // if the rightmost bit is 1 increment our counter
    if (value & 0x1) {
      count++;
    }
    value >>=1;
  }
  // all odd binaries end in 1
  return count & 0x1;
}
