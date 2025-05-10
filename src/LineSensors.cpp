#include "LineSensors.hpp"

LineSensor_t::LineSensor_t(uint8_t sensorPin){
    this->sensorPin = sensorPin;
}

void LineSensor_t::Setup(){
    pinMode(this->sensorPin, INPUT);
}

uint16_t LineSensor_t::Read(){
    return analogRead(this->sensorPin);
}
