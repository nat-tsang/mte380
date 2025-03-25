#include <Arduino.h>
#include "../include/Config.h"
#include "../src/Motor.cpp"
#include "../src/PixyLineTracker.cpp"
#include "../src/Helpers.cpp"
#include "../src/Fan.cpp"

// Example: Adjust pins and objects based on your setup
Motor leftMotor(u2_IN1, u2_IN2, true);
Motor rightMotor(u3_IN1, u3_IN2, false);

PixyLineTracker lineTracker;
Fan fan(FAN);

bool legoManAligned = false;

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
        
        if (x_error < thresholdX && y > thresholdY) {
          Serial.println("Legoman is centered, stopping");
          leftMotor.stop();
          rightMotor.stop();
          robotRunning = false;
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

void legoManAlignHelper(int thresholdX, int thresholdY) {
  // change this to if bullseye detected when that functionality works.
    while (!legoManAligned){
      auto [x, y] = lineTracker.getPixyCoord(5); // Lego man signature is 4
      Serial.print(x);
      Serial.print("\t");
      Serial.println(y);
      if (x > 0 && y > 0) {
        int x_error = X_CENTER - x;  // positive if legoman is to the left, negative if legoman is to the right
        int y_error = 140 - y; // 140 is the y value where the lego man should be located 
        
        if (abs(x_error) < thresholdX && y > thresholdY) {
          Serial.println("Legoman is centered, stopping");
          leftMotor.stop();
          rightMotor.stop();
          legoManAligned = true;
          return;
        } else {  // if legoman is not centered
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
    initButton(START_SIG);  // Initialize button pin
    fan.turnOff();
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