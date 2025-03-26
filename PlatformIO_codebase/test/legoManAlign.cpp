#include <Arduino.h>
#include "../include/Config.h"
#include "../src/Motor.cpp"
#include "../src/PixyLineTracker.cpp"
#include "../src/Helpers.cpp"
#include "../src/Fan.cpp"
#include "../src/ServoGripper.cpp"
#include "../src/TurnController.cpp"
#include "../src/EncoderReader.cpp"

// Example: Adjust pins and objects based on your setup
Motor leftMotor(u2_IN1, u2_IN2, true);
Motor rightMotor(u3_IN1, u3_IN2, false);

EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

PixyLineTracker lineTracker;
Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

bool legoManAligned = false;
bool hasTurned = false;

// void legoManAlign(int thresholdX, int thresholdY) {
//   // change this to if bullseye detected when that functionality works.
//     if (true){
//       auto [x, y] = lineTracker.getPixyCoord(5); // Lego man signature is 4
//       Serial.print(x);
//       Serial.print("\t");
//       Serial.println(y);
//       if (x > 0 && y > 0) {
//         // position lego man in center
//         int x_error = abs(X_CENTER - x);
//         int y_error = abs(Y_CENTER - y);
        
//         if (x_error < thresholdX && y > thresholdY) {
//           Serial.println("Legoman is centered, stopping");
//           leftMotor.stop();
//           rightMotor.stop();
//           robotRunning = false;
//           return;
//         } else {
//           int driveSpeed = y_error * LEGO_KPy;
//           int turnSpeed = x_error * LEGO_KPx;
//           leftMotor.setSpeed(constrain(driveSpeed + turnSpeed, 63, 255));
//           rightMotor.setSpeed(constrain(driveSpeed - turnSpeed, 63, 255));
//         }
//       } else {
//         // Lego man not detected, spin till in view 
//         leftMotor.stop();
//         rightMotor.stop();
//       }
//     }
// }

// void legoManAlignHelper(int thresholdX, int thresholdY) {
//   // change this to if bullseye detected when that functionality works.
//     while (!legoManAligned){
//       auto [x, y] = lineTracker.getPixyCoord(5); // Lego man signature is 4
//       Serial.print(x);
//       Serial.print("\t");
//       Serial.println(y);
//       if (x > 0 && y > 0) {
//         int x_error = X_CENTER - x;  // positive if legoman is to the left, negative if legoman is to the right
//         int y_error = 140 - y; // 140 is the y value where the lego man should be located 
        
//         if (abs(x_error) < thresholdX && y > thresholdY) {
//           Serial.println("Legoman is centered, stopping");
//           leftMotor.stop();
//           rightMotor.stop();
//           legoManAligned = true;
//           return;
//         } else {  // if legoman is not centered
//           int driveSpeed = y_error * LEGO_KPy;
//           int turnSpeed = x_error * LEGO_KPx;
//           leftMotor.setSpeed(constrain(driveSpeed + turnSpeed, 63, 255));
//           rightMotor.setSpeed(constrain(driveSpeed - turnSpeed, 63, 255));
//         }
//       } else {
//         // Lego man not detected, spin till in view 
//         leftMotor.stop();
//         rightMotor.stop();
//       }
//     }
// }

void setup() {
    Serial.begin(115200);
    lineTracker.begin();
    initButton(START_SIG);  // Initialize button pin
    gripper.attach(); // Attach servo at startup
    gripper.open();   // Close gripper  
    leftEncoder.reset(); // need these for 180 turn
    rightEncoder.reset(); // need these for 180 turn
    fan.turnOff();
}

void loop() {
  checkButton(leftMotor, rightMotor);
  if (robotRunning){
    leftMotor.stop();
    rightMotor.stop();
    gripper.close();
    if (!hasTurned){
      turnController.turnDegrees(180, 70); // 70 from testing in driveAndTurn.cpp
      hasTurned = true;
    }

  } else {
    hasTurned = false;
    gripper.open();
    leftMotor.stop();
    rightMotor.stop();
  }
    
}