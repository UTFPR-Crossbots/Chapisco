#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <Arduino.h>

class LineSensor_t{
    public:
        LineSensor_t(uint8_t sensorPin);
        void Setup();
        uint16_t Read();
    private:
        uint16_t sensorPin;
};

#endif