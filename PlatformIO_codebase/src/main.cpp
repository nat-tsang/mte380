/**
 * @brief This main loop runs the main state diagram for Piddy with steering detection
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

PixyLineTracker lineTracker; // Pixy object for line, bullseye and legoman detection

Filter errorFilter(0.2);

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

float targetVelocity = 1.3;  // m/s forward speed
float rightbasePWM = 75;  // Base PWM value
float leftbasePWM = 75;  // Base PWM value

enum PiddyState {
  LINE_FOLLOWING,
  BULLSEYE_DETECT,
  LEGOMAN_ALIGN,
  PICKUP_LEGOMAN,
  IDLE
};

PiddyState currentState = IDLE;

bool legoManAlign(int thresholdX, int thresholdY) {
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
      return true;
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
  return false;
}

void setup() {
  Serial.begin(115200);
  initButton(START_SIG);  // Initialize button pin
  leftEncoder.reset(); // need these for 180 turn
  rightEncoder.reset(); // need these for 180 turn
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
  if (robotRunning && currentState == IDLE) {
    currentState = LINE_FOLLOWING;
  } else if (!robotRunning && currentState != IDLE) {
    currentState = IDLE;
  }

  switch (currentState) {
    case IDLE:
      // do nothing, checkButton should ensure motors are already stopped
      break; 

    case LINE_FOLLOWING: {
      lineTracker.findBullseye(175, 50, 15, 20);
      if (lineTracker.isBullseye()) {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        Serial.println("Bullseye found in stopping range.");
        currentState = LEGOMAN_ALIGN;
        break;
      }

      if (!lineTracker.isLineDetected()) {
        leftMotor.stop();
        rightMotor.stop();
        linePID.reset();
        currentState = IDLE;
        robotRunning = false; // forces a manual reset on button
        break; 
      }
      
      int pixyError = lineTracker.readLinePosition();  // +160 (far left drift) to -160 (far right drift)
      int filteredError = errorFilter.computeSMA(pixyError);  // Using your Filter class
      Serial.print("filteredError: ");
      Serial.println(filteredError);

      float steeringCorrection = linePID.compute(filteredError);  // Output is differential m/s, -ve means turn left, +ve means turn right
      Serial.print("steeringCorrection: ");
      Serial.println(steeringCorrection); 

      float leftPWM = constrain(leftbasePWM + steeringCorrection, -150, 150);
      float rightPWM = constrain(rightbasePWM - steeringCorrection, -150, 150);
  
      // === Apply Motor Commands ===
      leftMotor.setSpeed(leftPWM);
      rightMotor.setSpeed(rightPWM);
      break;
    }
    case LEGOMAN_ALIGN: {
      if (legoManAlign(30, 150)) {
        Serial.println("Legoman centered. ");
        currentState = PICKUP_LEGOMAN;
      }
      break;
    }
    case PICKUP_LEGOMAN: {
      gripper.close();
      delay(1000); // debouncing, allows gripper to fully close 
      turnController.turnDegrees(180, 70); // 70 from testing in driveAndTurn.cpp
      // if the above turn has problems, definitely will need to edit turnController to turn until red line is found again or smth
      currentState = LINE_FOLLOWING;
      break;
    }
    default:
      // safety catch 
      leftMotor.stop();
      rightMotor.stop();
      currentState = IDLE;
      break;
  }
  delay(10); // ~ 100 Hz loop rate
}