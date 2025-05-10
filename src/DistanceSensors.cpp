#include "DistanceSensors.hpp"

const int VL53L0X_timingBudget = 20000;

void VL53L0X_setup(VL53L0X *sensor, uint16_t sensorPin, uint8_t address) {
    pinMode(sensorPin, OUTPUT);
    digitalWrite(sensorPin, LOW);

    delay(100);

    digitalWrite(sensorPin, HIGH);
    delay(10);

    sensor->setTimeout(500);

    sensor->setAddress(address);

    bool status = false;
    do{
        status = sensor->init();
    } while(!status);

    sensor->setMeasurementTimingBudget(VL53L0X_timingBudget);
}

uint16_t VL53L0X_read(VL53L0X *sensor) {
    uint16_t reading = sensor->readRangeSingleMillimeters();

    if (sensor->timeoutOccurred()){
        reading = 0xFFFF;
    }

    return reading;
}