/** 
 * @brief This file is the main control loop for Piddy 
 * WITH HELPERS, NO STATE MACHINE 
 */
#include <Arduino.h>
#include <Config.h>
#include <Motor.h>
#include <Fan.h>
#include <ServoGripper.h>
#include <EncoderReader.h>
#include <PixyLineTracker.h>
#include <Filter.h>
#include <PIDController.h>
#include <TurnController.h>
#include <Helpers.h>

// Bluetooth Serial
#define BTSerial Serial1

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);

Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

// Instantiate PID with gains from Config
PIDController linePID(LINE_KP, LINE_KI, LINE_KD);
PIDController leftVelocityPID(LEFT_VELOCITY_KP, LEFT_VELOCITY_KD, LEFT_VELOCITY_KI);
PIDController rightVelocityPID(RIGHT_VELOCITY_KP, RIGHT_VELOCITY_KD, RIGHT_VELOCITY_KI);

PixyLineTracker lineTracker; // signature for red line is 1
Pixy2 pixy;

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

float targetVelocity = 65;  //PWM forward 

void debugPrint(String msg) {
  Serial.println(msg);
  BTSerial.println(msg);
}

bool legoManAlign(int thresholdX, int thresholdY, const Block* block, int numBlock) {
  auto [x, y] = lineTracker.getPixyCoord(6, block, numBlock); // orange shayla is 6
  BTSerial.print(x);
  BTSerial.print("\t");
  BTSerial.println(y);
  if (x > 0 && y > 0) {
    int x_error = X_CENTER - x; // positive if legoman is to the left, negative if legoman is to the right
    // int y_error = thresholdY - y;  // If lego man is further, y is smaller. Therefore, y_error is larger.
    
    if (abs(x_error) < thresholdX && y > thresholdY) {  // TODO: What if lego man in close enough in y but not centered
      debugPrint("Legoman is centered, stopping");
      leftMotor.stop();
      rightMotor.stop();
      return true;
    } else {
      // int driveSpeed = y_error * LEGO_KPy;
      int turnSpeed = x_error * LEGO_KPx;    // Positive means turn right, negative means turn left
      leftMotor.setSpeed(60 - turnSpeed);
      rightMotor.setSpeed(60 + turnSpeed);  
    }
  } else {
    // Lego man not detected, spin till in view 
    leftMotor.stop();
    rightMotor.stop();
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  initButton(START_SIG);  // Initialize button pin
  leftEncoder.reset();
  rightEncoder.reset();
  linePID.reset();
  fan.turnOff(); // Turn fan off at startup
  gripper.attach(); // Attach servo at startup
  lineTracker.begin();
  // Close gripper & turn off fan
  gripper.close();
  fan.turnOff();
  debugPrint("Code uploaded.");
}

void loop() {
  checkButton(leftMotor, rightMotor);  // Check button state and toggle robotRunning state
  if (robotRunning) {
    pixy.ccc.getBlocks();
    const auto* blocks = pixy.ccc.blocks;
    int numBlocks = pixy.ccc.numBlocks;

    int pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +160 (far left drift) to -160 (far right drift)
    
    lineTracker.findBullseye(100, 40, 30, 20, blocks, numBlocks);
    if (lineTracker.getBullseye()) {
      leftMotor.stop();
      rightMotor.stop();
      robotRunning = false;
      lineTracker.setBullseye(false);      
      debugPrint("bullseye found in stopping range");
      // legoManAlign(30, 145, blocks, numBlocks);
    }
    if (!lineTracker.getLineDetected()) {
      leftMotor.stop();
      rightMotor.stop();
      robotRunning = false;
      linePID.reset();  // Optional: Reset PID when line is lost
      debugPrint("Line lost - stopping");
      return; // Skip the rest of the loop
    }

    // === Line Tracking Error from Pixy ===
    float steeringCorrection = linePID.compute(pixyError);  // Output is differential m/s, -ve means turn left, +ve means turn right

    float leftPWM = targetVelocity + steeringCorrection;
    float rightPWM = targetVelocity - steeringCorrection;

    // === Apply Motor Commands ===
    leftMotor.setSpeed(constrain(leftPWM, 0, 150));
    rightMotor.setSpeed(constrain(rightPWM, 0, 150));
    delay(10);  // 100 Hz update rate
  } 
  else {
    leftMotor.stop();
    rightMotor.stop();
  }
}
