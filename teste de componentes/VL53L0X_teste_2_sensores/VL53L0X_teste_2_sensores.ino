#include <Wire.h>
#include <VL53L0X.h>

// Define the pins for I2C communication
#define SCL_PIN 32
#define SDA_PIN 33

// VL53L0X Sensor control pins
#define LOX01_XSHUT_PIN 25
#define LOX02_XSHUT_PIN 27

// VL53L0X Sensor I2C addresses
#define LOX01_ADDRESS 0x30
#define LOX02_ADDRESS 0x31

VL53L0X lox01;
VL53L0X lox02;

void setup() {
  Serial.begin(9600);
  Wire.begin(SDA_PIN, SCL_PIN);

  delay(1000);
  Serial.println("Teste 2 VL53L0X");

  // Initialize sensor control pins
  pinMode(LOX01_XSHUT_PIN, OUTPUT);
  pinMode(LOX02_XSHUT_PIN, OUTPUT);

  // Start with both sensors turned off
  digitalWrite(LOX01_XSHUT_PIN, LOW);
  digitalWrite(LOX02_XSHUT_PIN, LOW);
  delay(10);

  // Initialize Sensor 1
  Serial.println("Initializing Sensor 1...");
  digitalWrite(LOX01_XSHUT_PIN, HIGH);
  delay(10);

  if (lox01.init()) {
    lox01.setAddress(LOX01_ADDRESS);
    Serial.println("Sensor 1 initialized and address set.");
  } else {
    while (1) {
      Serial.println("Failed to initialize Sensor 1");
      delay(1000);
    }
  }

  // Initialize Sensor 2
  Serial.println("Initializing Sensor 2...");
  digitalWrite(LOX02_XSHUT_PIN, HIGH);
  delay(10);

  if (lox02.init()) {
    lox02.setAddress(LOX02_ADDRESS);
    Serial.println("Sensor 2 initialized and address set.");
  } else {
    Serial.println("Failed to initialize Sensor 2");
    while (1) {
      Serial.println("Failed to initialize Sensor 2");
      delay(1000);
    }
  }

  // Start continuous measurements
  lox01.startContinuous(200);
  lox02.startContinuous(200);

  Serial.println("Both sensors are now taking continuous measurements.");
}

void loop() {
  int distance1 = lox01.readRangeContinuousMillimeters();
  int distance2 = lox02.readRangeContinuousMillimeters();

  if (lox01.timeoutOccurred()) {
    Serial.println("Sensor 1 Timeout!");
  } else {
    Serial.print("Sensor 1 Distance: ");
    Serial.print(distance1);
    Serial.println(" mm");
  }

  if (lox02.timeoutOccurred()) {
    Serial.println("Sensor 2 Timeout!");
  } else {
    Serial.print("Sensor 2 Distance: ");
    Serial.print(distance2);
    Serial.println(" mm");
  }

  delay(500);
}
