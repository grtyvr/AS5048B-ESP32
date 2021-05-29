#pragma once

#include <SPI.h>

class AS5048A{
    private:
        bool _errorFlag;
        uint8_t _cs;
        float _angle;  // stores the last angle read
        uint8_t _nullZone = 0;

    SPISettings settings;

    public:
        /**
         * @brief Constructor.
         * 
         *  @param {uint8_t} arg_cs The pin used for selecting the chip. 
         */
        AS5048A(uint8_t arg_cs, uint8_t nullZone = 3);
        /**
         * @brief Initializer sets up the SPI bus for communications with the chip.  
         * 
         * @param clock default value of 1KHz
         * @param bitOrder default value of MSBFIRST
         * @param dataMode default value of SPI_MODE1 
         */
        void init(uint32_t clock = 1000000, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE1);
        /**
         * Close the SPI bus.  If you need to work with other devices on the same bus that do not have the same paramaters
         * be sure to close the bus when you are done with a transaction.
         */
        void close();
        /**
         * getMagnitude
         * One of the diagnostic features of the AS5048 is that it can report the magnitude of the magnetic field
         * @return {uint16_t} magnitude.
         */
        uint16_t getMagnitude();
        /**
         * getAngle
         * @return {uint16_t} angle as a value in the interval [0,2^14-1].  Rotation counter clockwise 
         * from the current zero position
         */
        uint16_t getAngle();
        /**
         * getExpSmoothAngle
         * @param {float} smoothingFactor
         * @return {uint16_t} angle as a value in the interval [0,2^14-1].  Rotation counter clockwise
         * from the current zero position.  This value is caclulated from the last angle read and the 
         * current angle read by the following formula
         * new_angle = old_angle*(1 - smoothingFactor) + new_angle*smoothingFactor
         */
        uint16_t getExpSmoothAngle(float smoothingFactor);
        uint16_t getAverageAngle(int numSamples);
        void printDiagnostics();
        uint8_t getGain();
        uint8_t getErrors();
        bool error();
    private:
        uint16_t getDiag();
        uint8_t calcEvenParity(uint16_t value);
        uint16_t read(uint16_t registerAddress);
        uint16_t write(uint16_t registerAddress, uint16_t data);

};
