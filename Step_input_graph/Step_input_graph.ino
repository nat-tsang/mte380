#include <Encoder.h>

// Motor pins
const int LEFT_IN2 = 8;  // PWM2
const int LEFT_IN1 = 9;  // PWM3
const int RIGHT_IN2 = 6; // PWM4
const int RIGHT_IN1 = 7; // PWM5

// Encoder Pins
const int ENCODER_IN3 = 16; // Right
const int ENCODER_IN4 = 17; // Right
const int ENCODER_IN5 = 18; // Left
const int ENCODER_IN6 = 19; // Left

int inputPWM = 70;

Encoder rightEncoder(ENCODER_IN3, ENCODER_IN4);
Encoder leftEncoder(ENCODER_IN5, ENCODER_IN6);

long lastLeftPos = 0;
long lastRightPos = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  Serial.println("graphing encoder speed...");
}