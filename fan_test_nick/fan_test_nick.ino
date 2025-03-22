#include <Pixy2.h>

Pixy2 pixy;


const int START_SIG = 22; // Start Button

const int FAN_CTRL = 20;

bool go = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Serial is starting");

  pinMode(START_SIG, INPUT_PULLDOWN);
  pinMode(FAN_CTRL, OUTPUT);
  digitalWrite(FAN_CTRL, LOW);

}


void loop() {
  int buttonState = digitalRead(START_SIG);
  if (buttonState) {
    while(digitalRead(START_SIG)){
      Serial.println("Button still pressed. Take finger off.");
    }
    go = !go;
  }

  if (go) {
    digitalWrite(FAN_CTRL, HIGH);
    Serial.println("fan high");
  } else {
    digitalWrite(FAN_CTRL, LOW);
    Serial.println("fan low");
  }
}