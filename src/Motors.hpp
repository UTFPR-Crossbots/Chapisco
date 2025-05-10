#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

typedef enum MotorDirection_e{
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD
} MotorDirection_t;

class Motor_t{
    public:
        Motor_t(uint16_t pinIn1, uint16_t pinIn2);
        void Setup();
        void SetSpeed(uint8_t speed, MotorDirection_t direction);
    private:
        uint16_t pinIn1;
        uint16_t pinIn2;

};

#endif