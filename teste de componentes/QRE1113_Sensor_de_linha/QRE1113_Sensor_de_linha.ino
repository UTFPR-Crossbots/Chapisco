// QRE1113 Line Sensor
// 20/08/2024
// Marcus <marcus.vinicius.mvap@gmail.com>

const int LINE_SENSOR_PIN = 4;

void setup() {
  Serial.begin(115200);
  delay(5000);  // Await serial connection

  Serial.println("-- \t chapisco \t --");
  Serial.println("-- \t QRE1113 Line Sensor Test");
  Serial.println("-- \t " __FILE__);
  Serial.println("-- \t " __DATE__);

  pinMode(LINE_SENSOR_PIN, INPUT);
}

void loop() {
  int sensorValue = analogRead(LINE_SENSOR_PIN);

  Serial.print(F("Sensor Value: "));
  Serial.println(sensorValue);
  
  delay(100);
}
