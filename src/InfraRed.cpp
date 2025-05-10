#include "InfraRed.hpp"
#include <IRremote.hpp>

IrSensor_t::IrSensor_t(uint8_t sensorPin){
    this->sensorPin = sensorPin;
}

void IrSensor_t::Setup(){
    Serial.println(this->sensorPin);
    IrReceiver.begin(this->sensorPin);
}

uint16_t IrSensor_t::Read(){
    if(IrReceiver.decode()){
        this->command = IrReceiver.decodedIRData.command;
        IrReceiver.resume();
    }
    return this->command;
}
