#include <Arduino.h>
#include "../include/Config.h"
#include "../src/Motor.cpp"
#include "../src/EncoderReader.cpp"
#include "../src/TurnController.cpp"
#include "../src/PixyLineTracker.cpp"
#include "../src/Helpers.cpp"

// Example: Adjust pins and objects based on your setup
Motor leftMotor(u2_IN1, u2_IN2, true);
Motor rightMotor(u3_IN1, u3_IN2, false);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);
EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);

// 12cm wheelbase, 20mm wheel diameter
TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, 0.12, 0.02);

PixyLineTracker lineTracker;

bool start;

void legoManAlign(int thresholdX, int thresholdY) {
  // change this to if bullseye detected when that functionality works.
    if (true){
      auto [x, y] = lineTracker.getPixyCoord(5); // Lego man signature is 4
      Serial.print(x);
      Serial.print("\t");
      Serial.println(y);
      if (x > 0 && y > 0) {
        // position lego man in center
        int x_error = abs(X_CENTER - x);
        int y_error = abs(Y_CENTER - y);
        
        if (x_error < thresholdX && y < thresholdY) {
          Serial.println("Legoman is centered, stopping");
          leftMotor.stop();
          rightMotor.stop();
          start = false;
          return;
        } else {
          int driveSpeed = y_error * LEGO_KPy;
          int turnSpeed = x_error * LEGO_KPx;
          leftMotor.setSpeed(constrain(driveSpeed + turnSpeed, 63, 255));
          rightMotor.setSpeed(constrain(driveSpeed - turnSpeed, 63, 255));
        }
      } else {
        // Lego man not detected, spin till in view 
        leftMotor.stop();
        rightMotor.stop();
      }
    }
}

void setup() {
    Serial.begin(115200);
    lineTracker.begin();
}

void loop() {
  checkButton(leftMotor, rightMotor);

  if (robotRunning){
    legoManAlign(30, 150);
  } else {
    leftMotor.stop();
    rightMotor.stop();
  }
    
}