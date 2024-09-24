// DRV8871 Motor Driver Test
// 20/08/2024
// Marcus <marcus.vinicius.mvap@gmail.com>

#define MOTOR_L_IN1 18
#define MOTOR_L_IN2 19

#define MOTOR_R_IN1 21
#define MOTOR_R_IN2 22

void setup() {
  Serial.begin(115200);
  delay(5000); // Await serial connection

  Serial.println("-- \t chapisco \t --");
  Serial.println("-- \t DRV8871 Motor Driver Test");
  Serial.println("-- \t " __FILE__);
  Serial.println("-- \t " __DATE__);

  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
}

void MOTORS_write(int leftSpeed, int rightSpeed) {
  // Control left motor
  if (leftSpeed > 0) {
    analogWrite(MOTOR_L_IN1, leftSpeed);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else if (leftSpeed < 0) {
    analogWrite(MOTOR_L_IN2, -leftSpeed);
    digitalWrite(MOTOR_L_IN1, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
  }

  // Control right motor
  if (rightSpeed > 0) {
    analogWrite(MOTOR_R_IN1, rightSpeed);
    digitalWrite(MOTOR_R_IN2, LOW);
  } else if (rightSpeed < 0) {
    analogWrite(MOTOR_R_IN2, -rightSpeed);
    digitalWrite(MOTOR_R_IN1, LOW);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
  }
}

void loop() {

  for (int i = 100; i < 255; i++) {
    MOTORS_write(i, i);
    delay(100);
  }

  delay(500);

  for (int i = 100; i < 255; i++) {
    MOTORS_write(-i, -i);
    delay(100);
  }

  delay(500);
}
