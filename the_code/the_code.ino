#include <Arduino.h>
#include <Wire.h>
#include <IRremote.hpp>
#include <VL53L0X.h>

#define SCL_PIN 32
#define SDA_PIN 33

#define LOX_L_XSHUT 25
#define LOX_C_XSHUT 26
#define LOX_R_XSHUT 27

#define IR_RECEIVE_PIN 13

#define QRE_L 4
#define QRE_R 2

#define MOTOR_L_IN1 18
#define MOTOR_L_IN2 19
#define MOTOR_R_IN1 21
#define MOTOR_R_IN2 22

#define LED_R 12
#define LED_G 14
#define LED_B 23

enum State { IDLE, READY, COMBAT, STOP };
enum Strategy { UNDEFINED, AGGRESSIVE, DEFENSIVE };

struct Velocity { int left; int right; };



//============================================================//
//                      Robot Controller                      //
//============================================================//

class RobotController {

private:
    Velocity strategySpeed;
    State currentState;
    Strategy currentStrategy;

public:
    RobotController() : strategySpeed{0, 0}, currentState(IDLE), currentStrategy(UNDEFINED) {}

    void setState(State newState) { currentState = newState; }
    State getState() const { return currentState; }

    void setStrategy(Strategy strategy) { currentStrategy = strategy; }
    Strategy getStrategy() const { return currentStrategy; }

    void setSpeed(int leftSpeed, int rightSpeed) {
        strategySpeed.left = leftSpeed;
        strategySpeed.right = rightSpeed;
    }
    Velocity getSpeed() const { return strategySpeed; }
};


class MotorHandler {

public:
    void init() {
        pinMode(MOTOR_L_IN1, OUTPUT);
        pinMode(MOTOR_L_IN2, OUTPUT);
        pinMode(MOTOR_R_IN1, OUTPUT);
        pinMode(MOTOR_R_IN2, OUTPUT);
    }

    void write(int leftSpeed, int rightSpeed) {
        if (leftSpeed > 0) {
            analogWrite(MOTOR_L_IN1, leftSpeed);
            analogWrite(MOTOR_L_IN2, 0);
        } else {
            analogWrite(MOTOR_L_IN1, 0);
            analogWrite(MOTOR_L_IN2, -leftSpeed);
        }

        if (rightSpeed > 0) {
            analogWrite(MOTOR_R_IN1, rightSpeed);
            analogWrite(MOTOR_R_IN2, 0);
        } else {
            analogWrite(MOTOR_R_IN1, 0);
            analogWrite(MOTOR_R_IN2, -rightSpeed);
        }
    }

    void stop() {
        analogWrite(MOTOR_L_IN1, 0);
        analogWrite(MOTOR_L_IN2, 0);
        analogWrite(MOTOR_R_IN1, 0);
        analogWrite(MOTOR_R_IN2, 0);
    }
};



//============================================================//
//                        Lox Handler                         //
//============================================================//

class LoxHandler {

private:
    VL53L0X loxSensors[3];

public:
    void init() {
        pinMode(LOX_L_XSHUT, OUTPUT);
        pinMode(LOX_C_XSHUT, OUTPUT);
        pinMode(LOX_R_XSHUT, OUTPUT);

        digitalWrite(LOX_L_XSHUT, LOW);
        digitalWrite(LOX_C_XSHUT, LOW);
        digitalWrite(LOX_R_XSHUT, LOW);
        delay(10);

        digitalWrite(LOX_L_XSHUT, HIGH);
        delay(10);
        loxSensors[0].init();
        loxSensors[0].setTimeout(500);
        loxSensors[0].startContinuous();

        digitalWrite(LOX_C_XSHUT, HIGH);
        delay(10);
        loxSensors[1].init();
        loxSensors[1].setTimeout(500);
        loxSensors[1].startContinuous();

        digitalWrite(LOX_R_XSHUT, HIGH);
        delay(10);
        loxSensors[2].init();
        loxSensors[2].setTimeout(500);
        loxSensors[2].startContinuous();
    }

    int* getDistances() {
        static int distances[3];
        distances[0] = loxSensors[0].readRangeContinuousMillimeters();
        distances[1] = loxSensors[1].readRangeContinuousMillimeters();
        distances[2] = loxSensors[2].readRangeContinuousMillimeters();
        return distances;
    }
};



//============================================================//
//                          Core 00                           //
//============================================================//

class Core00 {

private:
    RobotController* controller;

    void move(int leftVelocity, int rightVelocity, int duration) {
        unsigned long targetTime = millis() + duration;
        controller->setSpeed(leftVelocity, rightVelocity);

        while (millis() < targetTime) {
            if (IrReceiver.decode() && IrReceiver.decodedIRData.command == 0x02) {
                controller->setState(STOP);
                controller->setSpeed(0, 0);
                break;
            }
        }
    }

    void strategyHandler() {
        Strategy strategy = controller->getStrategy();
        switch (strategy) {
            case UNDEFINED:
                move(255, 255, 20);
                move(255, 100, 10);
                break;
            case AGGRESSIVE:
                move(255, 255, 10);
                move(255, 0, 40);
                break;
            case DEFENSIVE:
                move(0, 0, 80);
                move(10, 10, 80);
                break;
        }
    }

    void loop() {
        if (IrReceiver.decode()) {
            switch (IrReceiver.decodedIRData.command) {
                case 0x00: Serial.println("Live signal"); break;
                case 0x01: controller->setState(COMBAT); break;
                case 0x02: controller->setState(STOP); break;
                case 0x04: controller->setStrategy(AGGRESSIVE); controller->setState(READY); break;
                case 0x05: controller->setStrategy(DEFENSIVE); controller->setState(READY); break;
                default: Serial.println("Unknown IR command received"); break;
            }
            IrReceiver.resume();
        }

        switch (controller->getState()) {
            case IDLE: break;
            case READY: digitalWrite(LED_R, HIGH); break;
            case COMBAT: strategyHandler(); break;
            case STOP: while (true) {} break;
        }
    }

public:
    Core00(RobotController* rc) : controller(rc) {}

    void task() {
        IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
        Serial.println("Core00 initialized");
        while (true) {
            loop();
        }
    }
};



//============================================================//
//                          Core 01                           //
//============================================================//

class Core01 {

private:
    MotorHandler motors;
    LoxHandler lox;
    RobotController* controller;

    void applySpeed() {
        Velocity speed = controller->getSpeed();
        motors.write(speed.left, speed.right);
    }

    void loop() {
        switch (controller->getState()) {
            case COMBAT:
                applySpeed();
                break;
            case IDLE:
            case READY:
                motors.stop();
                break;
            case STOP:
                motors.stop();
                while (true) {} // Stop everything in a loop
                break;
        }
    }

public:
    Core01(RobotController* rc) : controller(rc) {}

    void task() {
        motors.init();
        lox.init();
        Serial.println("Core01 (Motor and Lox) initialized");

        // Main task loop
        while (true) {
            loop();
        }
    }
};




//============================================================//
//                          __main__                          //
//============================================================//


RobotController robotController;
Core00 core00(&robotController);
Core01 core01(&robotController);


void strategyTask(void* parameter) { core00.task(); }
void motorAndLoxTask(void* parameter) { core01.task(); }

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    xTaskCreatePinnedToCore(strategyTask, "Strategy Task", 10000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motorAndLoxTask, "Motor and Lox Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
    // Main loop remains empty as tasks are handled by the cores
}
