#pragma once

#include <SPI.h>

class AS5048A{
    private:
    bool _errorFlag;
    uint8_t _cs;
    uint8_t _sck;
    uint8_t _copi;
    uint8_t _cipo;
    float _angle;

    SPISettings settings;

    public:
        AS5048A(uint8_t arg_cs);
        AS5048A(uint8_t arg_cs, uint8_t arg_sck, uint8_t arg_cipo, uint8_t arg_copi);
        void init();
        void close();
        uint16_t getMagnitude();
        uint16_t getAngle();
        uint16_t getExpSmoothAngle(float smoothingFactor);
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