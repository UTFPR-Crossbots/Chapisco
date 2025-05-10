#ifndef STRATEGY_H
#define STRATEGY_H

#include <Arduino.h>
#include "Motors.hpp"

typedef enum {
    STRATEGY_DIGITAL_HIGH = 0,
} StrategyTypes_t;

class Strategy_t{
    public:
        Strategy_t(Motor_t *motorLeft, Motor_t *motorRight);
        void SetStrategy(StrategyTypes_t strategy);
        void StrategyFunction(uint16_t leftReading, uint16_t centerReading, uint16_t rightReading, bool onLineLeft, bool onLineRight);
    private:
        uint8_t strategy;
        Motor_t *motorLeft;
        Motor_t *motorRight;
        void DigitalHigh(uint16_t leftReading, uint16_t centerReading, uint16_t rightReading, bool onLineLeft, bool onLineRight);
};

#endif