#include "Strategy.hpp"
#include "Motors.hpp"

Strategy_t::Strategy_t(Motor_t *motorLeft, Motor_t *motorRight) {
    this->strategy = 0;
    this->motorLeft = motorLeft;
    this->motorRight = motorRight;
}

void Strategy_t::SetStrategy(StrategyTypes_t strategy) {
    this->strategy = strategy;
}

void Strategy_t::StrategyFunction(uint16_t leftReading, uint16_t centerReading, uint16_t rightReading, bool onLineLeft, bool onLineRight){
    switch (strategy){
        case STRATEGY_DIGITAL_HIGH:
            this->DigitalHigh(leftReading, centerReading, rightReading, onLineLeft, onLineRight);
            break;
        
        default:
            this->DigitalHigh(leftReading, centerReading, rightReading, onLineLeft, onLineRight);
            break;
    }
}

void Strategy_t::DigitalHigh(uint16_t leftReading, uint16_t centerReading, uint16_t rightReading, bool onLineLeft, bool onLineRight) {
    this->motorLeft->SetSpeed(255, MOTOR_FORWARD);
    this->motorRight->SetSpeed(255, MOTOR_FORWARD);
}
