// VS1830B IR Receiver - Remote Control
// 20/08/2024
// Marcus <marcus.vinicius.mvap@gmail.com>

#include <IRremote.h>

const int IR_RECEIVE_PIN = 13;

void setup() {
  Serial.begin(9600);
  delay(5000);  // Await serial connection

  Serial.println("-- \t chapisco \t --");
  Serial.println("-- \t VS1830B IR Receiver Test");
  Serial.println("-- \t " __FILE__);
  Serial.println("-- \t " __DATE__);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals of protocols"));
  printActiveIRProtocols(&Serial);
}

void loop() {
  if (IrReceiver.decode()) {

    Serial.println("Decode IR signal");
    uint8_t command = NULL;

    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));

      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume();

    } else {

      IrReceiver.resume();
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      command = IrReceiver.lastDecodedCommand;

    }

    Serial.print("Comando recebido: ");
    Serial.println(command);

    if (command == 0x00) {
      Serial.println("Sinal de vida");
    } else if (0x01) {
      Serial.println("Começar");
    } else if (0x02) {
      Serial.println("Parar!");
    } else {
      Serial.println("Comando não definido");
    }

  }
}
