/**
 * @brief This main loop runs the main state diagram for Piddy with steering detection
 * THIS IS SWITCH CASE VERSION 
*/

#include <Arduino.h>
#include <Config.h>
#include <Motor.h>
#include <Fan.h>
#include <ServoGripper.h>
#include <PixyLineTracker.h>
#include <PIDController.h>
#include <Helpers.h>

<<<<<<<< HEAD:PlatformIO_codebase/src/main_bare_bones.cpp
========
#define BTSerial Serial1

EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

>>>>>>>> 8b2815d423c0a5c227ca81a2a890beaae69a6dab:PlatformIO_codebase/src/main_switch_cases.cpp.bak
Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);

Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

// Instantiate PID with gains from Config
PIDController linePID(LINE_KP, LINE_KI, LINE_KD);

PixyLineTracker lineTracker; // Pixy object for line, bullseye and legoman detection
Pixy2 pixy;

<<<<<<<< HEAD:PlatformIO_codebase/src/main_bare_bones.cpp
int rightbasePWM = 70;  // Base PWM value
int leftbasePWM = 70;  // Base PWM value
========
Filter<float, 3> pixyErrorFilter;    // For Pixy X-position (float)
Filter<float, 3> speedFilter;       // For encoder speeds (float)

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

float targetVelocity = 1.3;  // m/s forward speed
float rightbasePWM = 70;  // Base PWM value
float leftbasePWM = 70;  // Base PWM value
>>>>>>>> 8b2815d423c0a5c227ca81a2a890beaae69a6dab:PlatformIO_codebase/src/main_switch_cases.cpp.bak
bool hasTurned = false;

enum PiddyState {
  LINE_FOLLOWING,
  BULLSEYE_DETECT,
  // LEGOMAN_ALIGN,
  // PICKUP_LEGOMAN,
  IDLE
};

PiddyState currentState = IDLE;

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
  Serial.begin(115200);
  gripper.attach(); // Attach servo at startup
  gripper.close();   // Close gripper
  initButton(START_SIG);  // Initialize button pin

  linePID.reset();
  fan.turnOff(); // Turn fan off at startup
  lineTracker.begin();
  lineTracker.setLampOFF();
  debugPrint("Code uploaded. ");
}

void loop() {
  checkButton(leftMotor, rightMotor);  // Check button state and toggle robotRunning state
<<<<<<<< HEAD:PlatformIO_codebase/src/main_bare_bones.cpp
    if (robotRunning && currentState == IDLE) {
      gripper.close();
      currentState = LINE_FOLLOWING;
    } else if (!robotRunning && currentState != IDLE) {
      hasTurned = false;
      lineTracker.setLampOFF();
      currentState = IDLE;
    }
========
  if (robotRunning && currentState == IDLE) {
    gripper.open();
    currentState = LINE_FOLLOWING;
  } else if (!robotRunning && currentState != IDLE) {
    hasTurned = false;
    linePID.reset();
    pixyErrorFilter.reset();
    currentState = IDLE;
  }
>>>>>>>> 8b2815d423c0a5c227ca81a2a890beaae69a6dab:PlatformIO_codebase/src/main_switch_cases.cpp.bak

  pixy.ccc.getBlocks();
  const auto* blocks = pixy.ccc.blocks;
  int numBlocks = pixy.ccc.numBlocks;

  switch (currentState) {
    case IDLE:
      leftMotor.stop();
      rightMotor.stop();
      break; 

    case LINE_FOLLOWING: {
<<<<<<<< HEAD:PlatformIO_codebase/src/main_bare_bones.cpp
      // Serial.println("Line following now. ");
      float pixyError = lineTracker.readLinePosition();  // +157.5 (far left drift) to -157.5 (far right drift)

      lineTracker.findBullseye(100, 40, 30, 20); // currently on nightime, 175, 50, 30, 20 (daytime) 175, 50, 50, 20 (more forgiving x)
      if (lineTracker.isBullseye()) {
        leftMotor.stop();
        rightMotor.stop();
        Serial.println("Bullseye found in stopping range.");
        currentState = IDLE;
        break;
      }

      if (!lineTracker.isLineDetected()) {
        Serial.println("No line seen");
========
      float pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +157.5 (far left drift) to -157.5 (far right drift)
      
      lineTracker.findBullseye(100, 35, 50, 20, blocks, numBlocks); // currently on nightime, 175, 50, 30, 20 (daytime) 175, 50, 50, 20 (more forgiving x)
      if (lineTracker.getBullseye()) {
        leftMotor.stop();
        rightMotor.stop();
        debugPrint("Bullseye found in stopping range.");
        currentState = LEGOMAN_ALIGN;
        break;
      }

      if (!lineTracker.getLineDetected()) {
        debugPrint("No line seen");
        leftMotor.stop();
        rightMotor.stop();
        linePID.reset();
        pixyErrorFilter.reset();
>>>>>>>> 8b2815d423c0a5c227ca81a2a890beaae69a6dab:PlatformIO_codebase/src/main_switch_cases.cpp.bak
        currentState = IDLE;
        break; 
      }

      if (abs(pixyError) < 10.0) {
        pixyError = 0;
      }

      double steeringCorrection = linePID.compute(pixyError);  // Output is differential m/s, -ve means turn left, +ve means turn right

      int leftPWM = constrain(leftbasePWM + steeringCorrection, 0, 150);
      int rightPWM = constrain(rightbasePWM - steeringCorrection, 0, 150);
      
      // === Apply Motor Commands ===
      leftMotor.setSpeed(leftPWM);
<<<<<<<< HEAD:PlatformIO_codebase/src/main_bare_bones.cpp
      rightMotor.setSpeed(rightPWM);
      break;
    }

    // case LEGOMAN_ALIGN: {
    //   // lineTracker.setLampON();
    //   if (legoManAlign(30, 145)) {
    //     Serial.println("Legoman centered. ");
    //     // lineTracker.setLampOFF();
    //     currentState = PICKUP_LEGOMAN;
    //   }
    //   break;
    // }
    // case PICKUP_LEGOMAN: {
    //   gripper.close();
    //   // delay(1000); // debouncing, allows gripper to fully close 
    //   if (!hasTurned) {
    //     turnController.turnDegrees(180, 70); // 70 from testing in driveAndTurn.cpp
    //     hasTurned = true;
    //   }
    //   // if the above turn has problems, definitely will need to edit turnController to turn until red line is found again or smth
    //   // currentState = IDLE;
    //   lineTracker.setLampON();
    //   leftMotor.stop();
    //   rightMotor.stop();
    //   break;
    // }
========
      rightMotor.setSpeed(rightPWM);  
      break;
    }

    case LEGOMAN_ALIGN: {
      if (legoManAlign(30, 145, blocks, numBlocks)) {
        debugPrint("Legoman centered. ");
        currentState = PICKUP_LEGOMAN;
      }
      break;
    }
    case PICKUP_LEGOMAN: {
      gripper.close();
      debugPrint("Turning 180 with legoman. ");
      // delay(1000); // debouncing, allows gripper to fully close 
      if (!hasTurned) {
        turnController.turnDegrees(180, 70); // 70 from testing in driveAndTurn.cpp
        hasTurned = true;
      }
      leftMotor.stop();
      rightMotor.stop();
      break;
    }
>>>>>>>> 8b2815d423c0a5c227ca81a2a890beaae69a6dab:PlatformIO_codebase/src/main_switch_cases.cpp.bak
    default:
      // safety catch 
      leftMotor.stop();
      rightMotor.stop();
      currentState = IDLE;
      break;
  }
}