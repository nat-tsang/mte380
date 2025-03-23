#include <Arduino.h>
#include <Config.h>
#include <Motor.h>
#include <Fan.h>
#include <ServoGripper.h>
#include <EncoderReader.h>
#include <PixyLineTracker.h>
#include <Filter.h>
#include <PIDController.h>

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);
float rightMotorScale = 1.0;  // Default = no scaling

Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

// Instantiate PID with gains from Config
PIDController linePID(LINE_KP, LINE_KI, LINE_KD);

PixyLineTracker lineTracker(1); // signature for red line is 1

Filter errorFilter(0.2);

void calibrateMotors();  // Function prototype for calibrateMotors

void setup() {
  Serial.begin(115200);
  leftEncoder.reset();
  rightEncoder.reset();
  fan.turnOff(); // Turn fan off at startup
  gripper.attach(); // Attach servo at startup
  calibrateMotors();  // << Run calibration once
  lineTracker.begin();
}

void loop() {

  // Close gripper
  gripper.close();

  int error = lineTracker.readLinePosition();  // -160 (far left) to +160 (far right)
  int filteredError = errorFilter.computeSMA(error);  // Using your Filter class

  if (!lineTracker.isLineDetected()) {
    leftMotor.stop();
    rightMotor.stop();
    linePID.reset();  // Optional: Reset PID when line is lost
    Serial.println("Line lost - stopping");
    return;
  }

  float correction = linePID.compute(filteredError);

  leftMotor.setSpeed(constrain(CALIBRATION_PWM + correction, 63, 255)); // 63 is the minimum PWM value
  rightMotor.setSpeed(constrain((CALIBRATION_PWM - correction) * rightMotorScale, 63, 255));
  // rightMotor.setSpeed(CALIBRATION_PWM - correction);

  Serial.print("Base: "); Serial.print(CALIBRATION_PWM);
  Serial.print(" Correction: "); Serial.print(correction);
  Serial.print(" L: "); Serial.print(CALIBRATION_PWM - correction);
  Serial.print(" R: "); Serial.print((CALIBRATION_PWM + correction) * rightMotorScale);
  Serial.print(" R no scaling: "); Serial.println(CALIBRATION_PWM + correction);
  
  // Plotter-specific formatted line (starts with '>', uses var:value pairs)
  Serial.print(">");
  Serial.print("Error: ");
  Serial.print(filteredError);
  // Serial.print(",");
  // Serial.print("right pwm:");
  // Serial.print(rightMotor.getSpeed());
  // Serial.print(",");
  // Serial.print("left pwm:");
  // Serial.print(leftMotor.getSpeed());
  // Serial.print(",");
  // Serial.print("right encoder:");
  // Serial.print(rightEncoder.getTicks());
  // Serial.print(",");
  // Serial.print("left encoder:");
  // Serial.print(leftEncoder.getTicks());
  // Serial.print(",");
  // Serial.print("right speed (m/s):");
  // Serial.print(rightEncoder.computeSpeed());
  // Serial.print(",");
  // Serial.print("left speed (m/s):");
  // Serial.print(leftEncoder.computeSpeed());
  Serial.println();  // Auto appends \r\n
  delay(100);
}


void calibrateMotors() {
  delay(5000);  // Delay to allow serial monitor to open
  Serial.println("Starting Motor Calibration...");

  // Reset encoders
  leftEncoder.reset();
  rightEncoder.reset();

  // Run both motors forward
  leftMotor.setSpeed(CALIBRATION_PWM);
  rightMotor.setSpeed(CALIBRATION_PWM);

  // Wait for steady state
  delay(CALIBRATION_TIME);

  // Sample speed after running
  float leftSpeed = leftEncoder.computeSpeed();   // m/s
  float rightSpeed = rightEncoder.computeSpeed(); // m/s

  Serial.print("Left Speed (m/s): ");
  Serial.print(leftSpeed);
  Serial.print("\tRight Speed (m/s): ");
  Serial.println(rightSpeed);

  // Safety check to avoid divide by zero
  if (rightSpeed > 0) {
      rightMotorScale = leftSpeed / rightSpeed;
  } else {
      rightMotorScale = 1.0;
  }

  Serial.print("Calibration scale factor (right motor): ");
  Serial.println(rightMotorScale);

  // Stop motors after calibration
  leftMotor.stop();
  rightMotor.stop();
  delay(10000);
}

void legoManAlign() {
  if (lineTracker.isBullseye()){
    int x = lineTracker.getPixyX(4); // Lego man signature is 4
    if (x > 0) {
      // position lego man in center
    }
  }
}