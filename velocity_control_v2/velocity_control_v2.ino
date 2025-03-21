#include <Encoder.h>
#include "velocity_control.h"

const int ENCODER_IN3 = 16;
const int ENCODER_IN4 = 17;
const int ENCODER_IN5 = 14;
const int ENCODER_IN6 = 21;

const int u2_IN2 = 8;  // right
const int u2_IN1 = 9;
const int u3_IN2 = 6;
const int u3_IN1 = 7;  // left

Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6);  // Right
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4);  // Left

long prevLeftPos = 0;
long prevRightPos = 0;
int leftPWMOutput = 40;
int rightPWMOutput = 40;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  drive1Encoder.write(0);
  drive2Encoder.write(0);
  delay(7000);
}

void loop() {
  closedLoopVelocityControl(drive2Encoder, drive1Encoder, 
                            leftPWMOutput, rightPWMOutput, 
                            u3_IN1, u3_IN2, u2_IN1, u2_IN2, 
                            prevLeftPos, prevRightPos, lastTime);
}