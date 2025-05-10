#include <Arduino.h>
#include <Wire.h>
#include "LineSensors.hpp"
#include "InfraRed.hpp"
#include "DistanceSensors.hpp"
#include "Motors.hpp"
#include "Strategy.hpp"

//VL53L0X
const uint8_t LEFT_DISTANCE_SENSOR_PIN = 25;
const uint8_t CENTER_DISTANCE_SENSOR_PIN = 26;
const uint8_t RIGHT_DISTANCE_SENSOR_PIN = 27;

const uint8_t LEFT_DISTANCE_SENSOR_ADDRESS = 0x30;
const uint8_t CENTER_DISTANCE_SENSOR_ADDRESS = 0x31;
const uint8_t RIGHT_DISTANCE_SENSOR_ADDRESS = 0x32;

// VS1830B
const uint8_t IR_RECEIVE_PIN = 13;

// QRE1113
const uint8_t QRE_L = 2;
const uint8_t QRE_R = 4;

// DRV8871
const uint8_t MOTOR_L_IN1 = 18;
const uint8_t MOTOR_L_IN2 = 19;
const uint8_t MOTOR_R_IN1 = 21;
const uint8_t MOTOR_R_IN2 = 22;

// LEDs
const uint8_t LED_R = 12;
const uint8_t LED_G = 14;
const uint8_t LED_B = 23;

const uint16_t LINE_SENSOR_THRESHOLD = 300;

LineSensor_t leftLine(QRE_L);
LineSensor_t rightLine(QRE_R);

IrSensor_t infraRed(IR_RECEIVE_PIN);

VL53L0X leftDistance;
VL53L0X centerDistance;
VL53L0X rightDistance;

Motor_t motorLeft(MOTOR_L_IN1, MOTOR_L_IN2);
Motor_t motorRight(MOTOR_R_IN1, MOTOR_R_IN2);

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando...");
    Wire.begin();

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    // Inicializa os LEDs apagados
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);

    Serial.print("Motor left setup: ");
    motorLeft.Setup();
    Serial.println("ok");
    Serial.print("Motor right setup: ");
    motorRight.Setup();
    Serial.println("ok");

    Serial.print("Ir setup: ");
    infraRed.Setup();
    Serial.println("ok");

    Serial.print("Sensor left setup: ");
    VL53L0X_setup(&leftDistance, LEFT_DISTANCE_SENSOR_PIN, LEFT_DISTANCE_SENSOR_ADDRESS);
    Serial.println("ok");
    Serial.print("Sensor center setup: ");
    VL53L0X_setup(&centerDistance, CENTER_DISTANCE_SENSOR_PIN, CENTER_DISTANCE_SENSOR_ADDRESS);
    Serial.println("ok");
    Serial.print("Sensor right setup: ");
    VL53L0X_setup(&rightDistance, RIGHT_DISTANCE_SENSOR_PIN, RIGHT_DISTANCE_SENSOR_ADDRESS);
    Serial.println("ok");
}

void loop() {
    static Strategy_t strategy(&motorLeft, &motorRight);
    uint16_t command = infraRed.Read();
    static uint16_t lastCommand = 0xFF;

    bool onLeftLine = leftLine.Read() < LINE_SENSOR_THRESHOLD;
    bool onRightLine = rightLine.Read() < LINE_SENSOR_THRESHOLD;
    uint16_t leftDistanceRead = VL53L0X_read(&leftDistance);
    uint16_t centerDistanceRead = VL53L0X_read(&centerDistance);
    uint16_t rightDistanceRead = VL53L0X_read(&rightDistance);

    if(command != lastCommand){
        switch (command) {
            case 0:  // botão 1 do controle
                Serial.println("Sinal de vida");
                digitalWrite(LED_R, HIGH);
                delay(100);
                digitalWrite(LED_R, LOW);
                break;

            case 1:  // botão 2 do controle
                Serial.println("Iniciar combate");
                digitalWrite(LED_G, HIGH);
                strategy.StrategyFunction(0, 0, 0, onLeftLine, onRightLine);
                break;

            case 2:  // botão 3 do controle
                Serial.println("Parar");
                motorLeft.SetSpeed(0, MOTOR_FORWARD);
                motorRight.SetSpeed(0, MOTOR_FORWARD);
                break;

            case 3:  // botão 4 do controle
                strategy.SetStrategy(STRATEGY_DIGITAL_HIGH);
                break;

            default:
                Serial.print("Comando desconhecido: ");
                Serial.println(command, HEX);
                break;
        }
        lastCommand = command;
    }
}


