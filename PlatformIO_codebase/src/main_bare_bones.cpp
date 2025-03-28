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
#include <Filter.h>
#include <TurnController.h>
#include <EncoderReader.h>
// #define BTSerial Serial1

EncoderReader rightEncoder(ENCODER_IN6, ENCODER_IN5);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);

Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

// Instantiate PID with gains from Config
PIDController linePID(LINE_KP, LINE_KI, LINE_KD);

PixyLineTracker lineTracker; // Pixy object for line, bullseye and legoman detection
Pixy2 pixy;

Filter<float, 3> pixyErrorFilter;    // For Pixy X-position (float)

bool hasTurned = false;

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

enum PiddyState {
  LINE_FOLLOW_PICKUP,
  LINE_FOLLOW_DROPOFF,
  BULLSEYE_DETECT,
  LEGOMAN_ALIGN,
  PICKUP_LEGOMAN,
  UNLOAD_SHAYLA,
  GO_HOME,
  IDLE
};

PiddyState currentState = IDLE;

// === Function Prototype ===
void printVoltageLevel(int analogPin);
void debugPrint(String msg);
bool legoManAlign(int thresholdX, int thresholdY, const Block* block, int numBlock);
float checkBatteries(int pin);
void flashPixyLight(int times);


void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);

  gripper.attach(); // Attach servo at startup
  gripper.close();

  initButton(START_SIG);  // Initialize button pin
  linePID.reset();
  //fan.turnOn(); // Turn fan on at startup

  lineTracker.begin();  // Initialize Pixy
  lineTracker.setLampOFF();

  pinMode(BATTERY_LEVEL_PIN, INPUT); // Battery level

  debugPrint("Code uploaded. STARTING ROUTINE");
  checkBatteries(BATTERY_LEVEL_PIN);  // Check battery voltage
  delay(1000);
}


void loop() {
  if (SYSTEM_SHUTDOWN) return;  // Halt program here if shutdown triggered
  // === Read battery voltage ===
  checkBatteries(BATTERY_LEVEL_PIN);  // Check battery voltage
  
  static bool lastRobotRunning = false;
  // === Check button state ===
  checkButton(leftMotor, rightMotor);  // Check button state and toggle robotRunning state

  // === State toggle logic ===
  if (robotRunning != lastRobotRunning) { // If button state changes
    if (robotRunning) {
      debugPrint("Switching to LINE_FOLLOWING");
      gripper.open();  // Optional: ready for pickup
      lineTracker.setBullseye(false);
      currentState = LINE_FOLLOW_PICKUP;
      //currentState = LINE_FOLLOW_DROPOFF;   // FOR TESTING ONLY
    } else {
      debugPrint("Switching to IDLE");
      hasTurned = false;
      currentState = IDLE;
    }
  }
  lastRobotRunning = robotRunning;

  pixy.ccc.getBlocks();
  const auto* blocks = pixy.ccc.blocks;
  int numBlocks = pixy.ccc.numBlocks;

  switch (currentState) {
    case IDLE:
      leftMotor.stop();
      rightMotor.stop();
      break; 

    case LINE_FOLLOW_PICKUP: {
      float pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +157.5 (far left drift) to -157.5 (far right drift)

      // TODO: Check in Pixy if the following parameters are correct
      lineTracker.findBullseye(100, 20, 30, 20, blocks, numBlocks, leftMotor, rightMotor); // currently on nightime, 175, 50, 30, 20 (daytime) 175, 50, 50, 20 (more forgiving x)
      if (lineTracker.getBullseye()) {
        leftMotor.stop();
        rightMotor.stop();
        debugPrint("Bullseye found in stopping range. Setting to LEGOMAN_ALIGN");
        flashPixyLight(1);
        currentState = LEGOMAN_ALIGN;
        break;
      }
      if (!lineTracker.getLineDetected()) {
        debugPrint("No line seen in LINE_FOLLOW_PICKUP");
        leftMotor.stop();
        rightMotor.stop();
        linePID.reset();
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
      rightMotor.setSpeed(rightPWM);  
      break;
    }

    case LEGOMAN_ALIGN: {
      if (legoManAlign(20, 140, blocks, numBlocks)) {
        debugPrint("Legoman centered. Switching to PICKUP_LEGOMAN");
        currentState = PICKUP_LEGOMAN;
      }
      break;
    }

    case PICKUP_LEGOMAN: {
      delay(250);
      gripper.close();
      debugPrint("Turning 180 with legoman. ");
      delay(300); // debouncing, allows gripper to fully close 
      if (!hasTurned) {
        turnController.turnDegrees(-130, 70); // 70 from testing in driveAndTurn.cpp
        hasTurned = true;
        delay(250); // Allow robot to stop and not drit past red line?
      }
      lineTracker.readLinePosition(blocks, numBlocks);
      if (!lineTracker.getLineDetected()){
        turnController.turnDegrees(-30, 70);
      } else {
        lineTracker.setBullseye(false);
        linePID.reset();
        debugPrint("Switching to LINE_FOLLOW_DROPOFF");
        currentState = LINE_FOLLOW_DROPOFF;
      }
      break;
    }

    case LINE_FOLLOW_DROPOFF: {
      float pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +157.5 (far left drift) to -157.5 (far right drift)
      lineTracker.findGreenBox(0, 0, 0, 0, blocks, numBlocks); // passing in zeros because they are unused
      
      if (lineTracker.getGreenBox()) {    // TODO: Write function for turning and open servo and reverse and follow line home
        leftMotor.stop();
        rightMotor.stop();
        debugPrint("Green box found in stopping range. Setting to UNLOAD_SHAYLA");
        delay(1000);
        currentState = UNLOAD_SHAYLA;

        break;
      }
      
      if (!lineTracker.getLineDetected()) {
        debugPrint("No line seen in LINE_FOLLOW_DROPOFF");
        leftMotor.stop();
        rightMotor.stop();
        linePID.reset();
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
      rightMotor.setSpeed(rightPWM);  
      break;
    }

    case UNLOAD_SHAYLA: {
      debugPrint("Dropping off Shayla in green box");
      turnController.turnDegrees(40, 70); // Turn to face green box
      delay(1000); // Wait for turn to complete
      leftMotor.setSpeed(65); // Move robot forward
      rightMotor.setSpeed(65);
      delay(400); // Wait for robot to move forward
      leftMotor.stop();
      rightMotor.stop();
      delay(500); // Wait for robot to stop
      gripper.open();
      delay(500); // Wait for gripper to open
      leftMotor.setSpeed(-65); // Reverse robot
      rightMotor.setSpeed(-65);
      delay(500); // Wait for robot to reverse
      leftMotor.stop();
      rightMotor.stop();
      turnController.turnDegrees(40, -70); // Turn back to face line
      lineTracker.setGreenBox(false);
      debugPrint("Setting to GO_HOME");
      currentState = GO_HOME;
      break;
    }

    case GO_HOME: {
      float pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +157.5 (far left drift) to -157.5 (far right drift)

      if (!lineTracker.getLineDetected()) {
        debugPrint("No line seen in GO_HOME. Stopping");
        leftMotor.stop();
        rightMotor.stop();
        linePID.reset();
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
      rightMotor.setSpeed(rightPWM);  
      break;
    }

    default:
      // safety catch 
      leftMotor.stop();
      rightMotor.stop();
      currentState = IDLE;
      break;
  } 
}


// === Reads actual voltage using analog pin and divider ===
float checkBatteries(int pin) {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  float batteryVoltage = ((analogRead(pin)/ANALOG_RESOLUTION) * 3.3)/0.3197;

  // Only print every 5000 ms (5 seconds)
  if (currentTime - lastPrintTime >= 5000) {
    debugPrint("Battery Voltage: " + String(batteryVoltage));
    lastPrintTime = currentTime;
  }

  if (batteryVoltage < BATTERY_VOLTAGE_THRESHOLD) {
    debugPrint("Battery low. SYSTEM SHUTDOWN");
    SYSTEM_SHUTDOWN = true;
    currentState = IDLE;
    leftMotor.stop();
    rightMotor.stop();
    fan.turnOff();
    flashPixyLight(5);
  }
  return batteryVoltage;
}

// === Flashes Pixy cam lamp ===
void flashPixyLight(int times) {
  for (int i = 0; i < times; i++) {
    pixy.setLamp(1, 1); // LEDs ON
    delay(200);
    pixy.setLamp(0, 0); // LEDs OFF
    delay(200);
  }
}

bool legoManAlign(int thresholdX, int thresholdY, const Block* block, int numBlock) {     // TODO: Do we need to call get blocks again?
  auto [x, y] = lineTracker.getPixyCoord(6, block, numBlock); // orange shayla is 6,  TODO: Put in config
  debugPrint("Shayla x: " + String(x) + "   |   y: " + String(y));

  if (x >= 0 && y >= 0) {
    int x_error = X_CENTER - x; // positive if legoman is to the left, negative if legoman is to the right
    debugPrint("Shayla x_error: " + String(x_error));

    if (abs(x_error) < thresholdX && y > thresholdY) { 
      debugPrint("Shayla is centered, stopping");
      leftMotor.stop();
      rightMotor.stop();
      return true;
    } 
    else {
      if (abs(x_error) < thresholdX) { 
        debugPrint("Shayla is centered");
        // drive forward only once Shayla is centered
        if (y < thresholdY) { // Legoman is too far, drive forward slowly
          debugPrint("Shayla is too far");
          leftMotor.setSpeed(70);
          rightMotor.setSpeed(70);
          debugPrint("Left PWM: " + String(leftMotor.getSpeed()) + "  |   Right PWM: " + String(rightMotor.getSpeed()));
          delay(100);
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          delay(200);
        }
      }
      else if (abs(x_error) > thresholdX) {    // Turn robot on it's center axis
        debugPrint("Shayla is not centered");
        //int turnSpeed = abs(x_error * LEGO_KPx);    // Positive means turn right, negative means turn left
        if (x_error > 0) {   // Positive means turns left  
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(70);
          delay(100);
          debugPrint("Left PWM: " + String(leftMotor.getSpeed()) + "  |   Right PWM: " + String(rightMotor.getSpeed()));
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          delay(200); 

        }
        else if (x_error < 0) {  // Negative means turns right
          leftMotor.setSpeed(70);
          rightMotor.setSpeed(0);
          debugPrint("Left PWM: " + String(leftMotor.getSpeed()) + "  |   Right PWM: " + String(rightMotor.getSpeed()));
          delay(100);
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          delay(200);
        }
      }
    }
  } else {   // Lego man not detected, TODO: spin till in view 
    debugPrint("Shayla is not found, pulsing forward");
    leftMotor.setSpeed(70);
    rightMotor.setSpeed(70);
    debugPrint("Left PWM: " + String(leftMotor.getSpeed()) + "  |   Right PWM: " + String(rightMotor.getSpeed()));
    delay(100);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(200);
  }
  return false;
}
