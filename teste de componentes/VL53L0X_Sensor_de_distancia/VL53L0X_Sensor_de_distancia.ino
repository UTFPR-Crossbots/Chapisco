// Multiple VL53L0X Sensors with Custom I2C Addresses
// 20/08/2024
// Marcus <marcus.vinicius.mvap@gmail.com>

#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 33
#define SCL_PIN 32

const int VL53L0X_count = 3;
const int VL53L0X_pin[VL53L0X_count] = { 25, 26, 27 };
const int VL53L0X_address[VL53L0X_count] = { 0x30, 0x31, 0x32 };

VL53L0X sensors[VL53L0X_count];

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing VL53L0X sensors...");

  // Initialize sensor pins
  for (int i = 0; i < VL53L0X_count; i++) {
    pinMode(VL53L0X_pin[i], OUTPUT);
    digitalWrite(VL53L0X_pin[i], LOW);
  }
  delay(10);

  // Power up and initialize each sensor
  for (int i = 0; i < VL53L0X_count; i++) {
    Serial.print("Powering up sensor ");
    Serial.println(i + 1);
    digitalWrite(VL53L0X_pin[i], HIGH);
    delay(10);
    
    if (sensors[i].init()) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" initialized successfully.");
    } else {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" initialization failed.");
      while (1);  // Halt the program if initialization fails
    }
    
    sensors[i].setAddress(VL53L0X_address[i]);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.println(" address set.");
    
    sensors[i].startContinuous(200);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.println(" started continuous measurement.");
  }

  Serial.println("All VL53L0X sensors initialized.");
}

void loop() {
  for (int i = 0; i < VL53L0X_count; i++) {
    uint16_t distance = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": Timeout!\t");
    } else {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distance);
      Serial.print(" mm\t");
    }
  }
  Serial.println();
  delay(200);
}
