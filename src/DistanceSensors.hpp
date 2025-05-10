#ifndef COMPONENTS_H
#define COMPONENTS_H
#include <VL53L0X.hpp>
#include <Arduino.h>

void VL53L0X_setup(VL53L0X *sensor, uint16_t sensorPin, uint8_t address);
uint16_t VL53L0X_read(VL53L0X *sensor);

#endif