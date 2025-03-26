#include <Arduino.h>
// Bluetooth Serial
#define BTSerial Serial1

// PID Constants
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.08;

// Example sensor pins (adjust based on your sensors)
const int leftSensor = A0;
const int rightSensor = A1;

// Motor control pins
const int leftMotor = 5;
const int rightMotor = 6;

// Bluetooth Serial (adjust to Serial1, Serial2, etc., based on wiring)
#define BTSerial Serial1

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Line threshold (calibrate based on your sensor readings)
const int threshold = 500;

void setup() {
  Serial.begin(115200);      // For debugging
  BTSerial.begin(9600);      // HC-05 default baud rate

  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
}

void loop() {
  // Read line sensors
  int leftValue = analogRead(leftSensor);
  int rightValue = analogRead(rightSensor);

  // Simple error calculation: negative if veering right, positive if veering left
  error = (leftValue - rightValue);

  // PID calculations
  integral += error;
  derivative = error - previousError;
  correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Motor speed calculations
  int baseSpeed = 150;  // Adjust based on your motors
  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);

  analogWrite(leftMotor, leftSpeed);
  analogWrite(rightMotor, rightSpeed);

  // Non-blocking Bluetooth check
  if (BTSerial.available()) {
    String btData = BTSerial.readStringUntil('\n');  // Read incoming line
    Serial.print("Received from Bluetooth: ");
    Serial.println(btData);

    // Example: change Kp dynamically via Bluetooth
    if (btData.startsWith("KP:")) {
      Kp = btData.substring(3).toFloat();
      Serial.print("Updated Kp: ");
      Serial.println(Kp);
    }
  }

  delay(10); // Small delay for stability
}