#include <Encoder.h>

// Motor pins
const int LEFT_IN2 = 6;  // PWM2
const int LEFT_IN1 = 7;  // PWM3
const int RIGHT_IN2 = 8; // PWM4
const int RIGHT_IN1 = 9; // PWM5

// Encoder Pins
const int ENCODER_IN3 = 16; // Drive 2 connected to U3
const int ENCODER_IN4 = 17; // Drive 2
const int ENCODER_IN5 = 14; // Drive 1 NEW PINS (right) connected to U2
const int ENCODER_IN6 = 21; // Drive 1 NEW PINS

int inputPWM = 60;

Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6);    // This will also set the pins as INPUT
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4);  // left


long lastLeftPos = 0;
long lastRightPos = 0;
unsigned long startTime;
unsigned long lastTime = 0;
const unsigned long interval = 100;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  Serial.println("graphing encoder speed...");
  startTime = millis();
  drive1Encoder.write(0);
  drive2Encoder.write(0);

  delay(2000);
  
  moveFwd(inputPWM);
}

void loop() {
  unsigned long currentTime = millis() - startTime;

  if (currentTime - lastTime >= interval) {
    long leftPos = drive2Encoder.read();
    long rightPos = drive1Encoder.read();

    // Compute speed (counts per interval)
    float leftSpeed = (leftPos - lastLeftPos) / (interval / 1000.0);  // counts per second
    float rightSpeed = (rightPos - lastRightPos) / (interval / 1000.0); // counts per second

    // Send data to Arduino Serial Plotter (formatted correctly)
    Serial.print(currentTime); Serial.print(",");
    Serial.print(inputPWM); Serial.print(",");
    Serial.print(leftSpeed); Serial.print(",");
    Serial.println(rightSpeed);

    // Store last values
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    lastTime = currentTime;
  }
  
}

void moveFwd(int speed){    // Move Left and RIGHT wheel forward (IN1 = HIGH, IN2 = LOW)
  analogWrite(LEFT_IN1, speed);
  analogWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}