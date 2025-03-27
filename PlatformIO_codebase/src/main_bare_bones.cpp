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

int rightbasePWM = 70;  // Base PWM value
int leftbasePWM = 70;  // Base PWM value
bool hasTurned = false;

TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, WHEEL_BASE, WHEEL_DIAMETER);

enum PiddyState {
  LINE_FOLLOW_PICKUP,
  LINE_FOLLOW_DROPOFF,
  BULLSEYE_DETECT,
  LEGOMAN_ALIGN,
  PICKUP_LEGOMAN,
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
      lineTracker.findBullseye(100, 35, 50, 20, blocks, numBlocks); // currently on nightime, 175, 50, 30, 20 (daytime) 175, 50, 50, 20 (more forgiving x)
      if (lineTracker.getBullseye()) {
        leftMotor.stop();
        rightMotor.stop();
        delay(500);
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

    case LINE_FOLLOW_DROPOFF: {
      debugPrint("Going home!");
      float pixyError = lineTracker.readLinePosition(blocks, numBlocks);  // +157.5 (far left drift) to -157.5 (far right drift)

      if (!lineTracker.getLineDetected()) {
        debugPrint("No line seen");
        leftMotor.stop();
        rightMotor.stop();
        gripper.open();
        linePID.reset();
        pixyErrorFilter.reset();
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
      if (legoManAlign(30, 135, blocks, numBlocks)) {
        debugPrint("Legoman centered. ");
        currentState = PICKUP_LEGOMAN;
      }
      break;
    }

    case PICKUP_LEGOMAN: {
      gripper.close();
      debugPrint("Turning 180 with legoman. ");
      delay(500); // debouncing, allows gripper to fully close 
      if (!hasTurned) {
        turnController.turnDegrees(-130, 70); // 70 from testing in driveAndTurn.cpp
        hasTurned = true;
        delay(500); // Allow robot to stop and not drit past red line?
      }
      // leftMotor.stop();
      // rightMotor.stop();
      lineTracker.setBullseye(false);
      currentState = LINE_FOLLOW_DROPOFF;
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


// void debugPrint(String msg) {
//   Serial.println(msg);
//   BTSerial.println(msg);
// }


bool legoManAlign(int thresholdX, int thresholdY, const Block* block, int numBlock) {
  auto [x, y] = lineTracker.getPixyCoord(6, block, numBlock); // orange shayla is 6
  BTSerial.print("Shayla x: " + String(x));
  BTSerial.println ("   |   y: " + String(y));
  if (x > 0 && y > 0) {
    int x_error = X_CENTER - x; // positive if legoman is to the left, negative if legoman is to the right
    // int y_error = thresholdY - y;  // If lego man is further, y is smaller. Therefore, y_error is larger.
    BTSerial.print("Shayla ERROR: " + String(x_error));

    if (abs(x_error) < thresholdX && y > thresholdY) {  // TODO: What if lego man in close enough in y but not centered
      debugPrint("Legoman is centered, stopping");
      leftMotor.stop();
      rightMotor.stop();
      return true;
    } else {
      // int driveSpeed = y_error * LEGO_KPy;
      int turnSpeed = x_error * LEGO_KPx;    // Positive means turn right, negative means turn left
      
      BTSerial.print("Left PWM: " + String(60 - turnSpeed));
      BTSerial.println("  |   Right PWM: " + String(60 + turnSpeed));
      leftMotor.setSpeed(constrain(60 - turnSpeed, 60, 150));
      rightMotor.setSpeed(constrain(60 + turnSpeed, 60, 150));  
    }
  } else {
    // Lego man not detected, spin till in view 
    leftMotor.stop();
    rightMotor.stop();
  }
  return false;
}