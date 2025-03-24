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
PIDController leftVelocityPID(LEFT_VELOCITY_KD, LEFT_VELOCITY_KD, LEFT_VELOCITY_KI);
PIDController rightVelocityPID(RIGHT_VELOCITY_KP, RIGHT_VELOCITY_KD, RIGHT_VELOCITY_KI);

PixyLineTracker lineTracker; // signature for red line is 1

Filter errorFilter(0.2);

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

float targetVelocity = 1.3;  // m/s forward speed
float rightbasePWM = RIGHT_KF * targetVelocity;  // Feedforward term
float leftbasePWM = LEFT_KF * targetVelocity;  // Feedforward term

void setup() {
  Serial.begin(115200);
  initButton(START_SIG);  // Initialize button pin
  leftEncoder.reset();
  rightEncoder.reset();
  linePID.reset();
  fan.turnOff(); // Turn fan off at startup
  gripper.attach(); // Attach servo at startup
  // calibrateMotors();  // << Run calibration once
  lineTracker.begin();

  // Close gripper & turn off fan
  gripper.close();
  fan.turnOff();
}

void loop() {
  checkButton(leftMotor, rightMotor);  // Check button state and toggle robotRunning state
  if (robotRunning) {
    int pixyError = lineTracker.readLinePosition();  // +160 (far left drift) to -160 (far right drift)
    int filteredError = errorFilter.computeSMA(pixyError);  // Using your Filter class
    Serial.print("filteredError: ");
    Serial.println(filteredError);

    if (!lineTracker.isLineDetected()) {
      leftMotor.stop();
      rightMotor.stop();
      linePID.reset();  // Optional: Reset PID when line is lost
      Serial.println("Line lost - stopping");
      return; // Skip the rest of the loop
    }

    // === Filtered Encoder Speed Measurements ===
    float leftSpeed = leftEncoder.computeSpeed();   // m/s
    float rightSpeed = rightEncoder.computeSpeed(); // m/s

    // === Line Tracking Error from Pixy ===
    float steeringCorrection = linePID.compute(filteredError);  // Output is differential m/s, -ve means turn left, +ve means turn right
    Serial.print("steeringCorrection: ");
    Serial.println(steeringCorrection); 
    
    // === Compute target wheel velocities ===
    float targetLeftVel = targetVelocity + steeringCorrection; // 0.3 m/s + correction
    float targetRightVel = targetVelocity - steeringCorrection;

    // === Per-Motor Velocity PID === The PID loop is designed to output the PWM correction needed to reach the target velocity
    // float leftPWM = leftVelocityPID.compute(targetLeftVel - leftSpeed);
    // float rightPWM = rightVelocityPID.compute(targetRightVel - rightSpeed);
    float leftPIDOutput = leftVelocityPID.compute(targetVelocity - leftSpeed);
    float rightPIDOutput = rightVelocityPID.compute(targetVelocity - rightSpeed);
    float leftPWM = leftbasePWM + leftPIDOutput;
    float rightPWM = rightbasePWM + rightPIDOutput;

    // === Apply Motor Commands ===
    leftMotor.setSpeed(leftPWM);
    rightMotor.setSpeed(rightPWM);


    Serial.print(">");    // Plotter-specific formatted line (starts with '>', uses var:value pairs)
    // Serial.print("Pixy Error: ");
    // Serial.print(pixyError);
    // Serial.print(",");
    // Serial.print("targetLeftVel:");
    // Serial.print(targetLeftVel);
    // Serial.print(",");
    // Serial.print("targetRightVel:");
    // Serial.print(targetRightVel);
    // Serial.print(",");
    Serial.print("leftpwm:");
    Serial.print(leftPWM);
    Serial.print(", rightpwm:");
    Serial.print(rightPWM);
    Serial.print(", leftspeedmpers:");
    Serial.print(leftSpeed);
    Serial.print(", rightspeedmpers:");
    Serial.println(rightSpeed);
    delay(10);  // 100 Hz update rate
  } 
  // else {
  //   Serial.println("Buton pressed, robot is stopped");
  // }
}

void legoManAlign(int thresholdX, int thresholdY) {
  Serial.println("aligning legoman");
  if (lineTracker.isBullseye()){
      
    auto [x, y] = lineTracker.getPixyCoord(5); // Lego man signature is 4
    if (x > 0 && y > 0) {
      Serial.println("positioning lego man");
      // position lego man in center
      int x_error = abs(X_CENTER - x);
      int y_error = abs(Y_CENTER - y);
      
      if (x_error < thresholdX && y_error < thresholdY) {
        leftMotor.stop();
        rightMotor.stop();
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