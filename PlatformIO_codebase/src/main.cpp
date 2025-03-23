#include <Arduino.h>
#include <Config.h>
#include <Motor.h>
#include <Fan.h>
#include <ServoGripper.h>
#include <EncoderReader.h>
#include <PixyLineTracker.h>
#include <Filter.h>

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);
float rightMotorScale = 1.0;  // Default = no scaling

Fan fan(FAN);
ServoGripper gripper(SERVO, minPulse, maxPulse);

// signature for red line is 1
PixyLineTracker lineTracker(1);

Filter errorFilter(0.2);

void calibrateMotors();  // Function prototype for calibrateMotors

void setup() {
  Serial.begin(9600);
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

  int baseSpeed = 80;
  int error = lineTracker.readLinePosition();  // -160 (far left) to +160 (far right)
  int filteredError = errorFilter.computeSMA(error);  // Using your Filter class

  // Basic P-control
  float Kp = 0.05;
  int correction = Kp * filteredError;

  leftMotor.setSpeed(baseSpeed - correction);
  rightMotor.setSpeed((baseSpeed + correction) * rightMotorScale);

  Serial.print(">");
  // Plotter-specific formatted line (starts with '>', uses var:value pairs)
  Serial.print(">");
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print(",");
  Serial.print("right encoder:");
  Serial.print(rightEncoder.getTicks());
  Serial.print(",");
  Serial.print("left encoder:");
  Serial.print(leftEncoder.getTicks());
  Serial.print(",");
  Serial.print("right speed (m/s):");
  Serial.print(rightEncoder.computeSpeed());
  Serial.print(",");
  Serial.print("left speed (m/s):");
  Serial.print(leftEncoder.computeSpeed());
  Serial.println();  // Auto appends \r\n
  delay(50);
}


void calibrateMotors() {
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
  delay(500);
}