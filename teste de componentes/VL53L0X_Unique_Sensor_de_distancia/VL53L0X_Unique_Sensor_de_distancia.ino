#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// Define the pins for I2C communication
#define SCL_PIN 32
#define SDA_PIN 33

void setup() {
  Serial.begin(9600);

  Serial.println("Unico VL53L0X Sensor de distancia teste");
  Wire.begin(SDA_PIN, SCL_PIN);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    while (1) {
      Serial.println("Failed to detect and initialize sensor!");
      delay(1000);
    }
  }
}

void loop() {
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}
