#ifndef INFRA_RED_H
#define INFRA_RED_H

#include <Arduino.h>

class IrSensor_t{
    public:
        IrSensor_t(uint8_t sensorPin);
        void Setup();
        uint16_t Read();
    private:
        uint8_t sensorPin;
        uint16_t command;
};

#endif