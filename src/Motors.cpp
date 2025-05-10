#include "Motors.hpp"

Motor_t::Motor_t(uint16_t pinIn1, uint16_t pinIn2) {
    this->pinIn1 = pinIn1;
    this->pinIn2 = pinIn2;
}

void Motor_t::Setup(){
    pinMode(this->pinIn1, OUTPUT);
    pinMode(this->pinIn2, OUTPUT);
  
    digitalWrite(this->pinIn1, LOW);
    digitalWrite(this->pinIn2, LOW);
}
  
void Motor_t::SetSpeed(uint8_t speed, MotorDirection_t direction) {
    if(direction == MOTOR_FORWARD){
        analogWrite(this->pinIn1, speed);
        digitalWrite(this->pinIn2, LOW);
    }
    else{
        digitalWrite(this->pinIn1, LOW);
        analogWrite(this->pinIn2, speed);
    }
}
