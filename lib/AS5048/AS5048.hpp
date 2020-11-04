#pragma once

#include <SPI.h>

class AS5048A{
    private:
    bool _errorFlag;
    uint8_t _cs;
    float _angle;  // stores the last angle read

    SPISettings settings;

    public:
        AS5048A(uint8_t arg_cs);
        void init();
        void close();
        uint16_t getMagnitude();
        uint16_t getAngle();
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