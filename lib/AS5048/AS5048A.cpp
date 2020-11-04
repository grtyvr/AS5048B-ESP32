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

AS5048A::AS5048A(uint8_t arg_cs){
  _cs = arg_cs;
  pinMode(_cs, OUTPUT);
  _errorFlag = false;
  _angle = 0.0;                           // initialize to zero degrees
}

void AS5048A::init(){
  // set up the SPI interface so that we can communicate with the chip
  // use mode 1, 
  settings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
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
  return rawData &= 0x3FFF;
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

uint16_t AS5048A::getAverageAngle(int numSamples){
  uint16_t retVal;
  // define a constant for the smallest angular increment we can have in radians
  // to generalize this to other bitdepth encoders replace the 2^14 with the number of increments
  const float angleIncrement = ( 2 * PI ) / (2^14);
  /// take a number of samples and return the circular mean value
  /// since we might have a situation where we are sampling at the transition from
  /// 2^14 - n to m for small integer n and small integer m, have to be carefull to normalize our data
  uint16_t sample;
  float x_coord = 0;
  float y_coord = 0;
  for (int i=0; i < numSamples; i++){
    // take a sample and compute the x and y coordinate of the sample as if it on the unit circle
    sample = getAngle();
    x_coord += cos(sample * angleIncrement);
    y_coord += sin(sample * angleIncrement);
  }
  // Take the average X and average Y values
  float averageX = x_coord/numSamples;
  float averageY = y_coord/numSamples;
  if ( (averageX == 0.0) & (averageY ==0.0) ){
    // pathalogical case of both X and Y being equal to zero as floats!
    retVal = 0;
  } else {
    float angle = atan2(averageY, averageX);
    if (angle >= 0){
      // if the angle returned by atan2 is positive then it is a positive rotation CCW 
      retVal = round(angle/angleIncrement);
    }

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
  return read(AS5048_REG_AGC) &0xFF;
}

uint8_t AS5048A::getErrors(){
  // read the error register of the last command
  uint8_t retVal;
  // Set up the command we will send
  uint16_t command = 0x4001;  // 0b0100000000000001
  vspi->beginTransaction(settings);
  // Drop cs low to enable the AS5048
  digitalWrite(_cs, LOW);
  retVal = vspi->transfer16(command);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cs, LOW);
  // you have to poll the chip twice.  Data from previous command comes
  // back on the next SPI transfer
  retVal = vspi->transfer16(command);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cs, LOW);
  // // you have to poll the chip twice.  Data from previous command comes
  // // back on the next SPI transfer
  vspi->transfer16(command);
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
