// #include <PID_v1.h>
// Follows tail of line
#include <Pixy2.h>
#include <Servo.h>

Servo myServo;
Pixy2 pixy;
const int FAN = 20;
const int u2_IN2 = 8; 
const int u2_IN1 = 9; 
const int u3_IN2 = 6;
const int u3_IN1 = 7; // left 

const int START_SIG = 22; // Pin 22 is connected to button
const int SERVO = 5;

const int minPulse = 700;   // microseconds for ~0 degrees
const int maxPulse = 2000;  // microseconds for ~120 degrees

// PID constants (tune these values based on your robot's behavior)
double Kp = 0.5;  // Proportional gain
double Ki = 0.0;  // Integral gain (may not be needed for line following)
double Kd = 0.08;  // Derivative gain (start at 0 before tuning)

// PID variables
double setpoint = 157.5;       // Center of Pixy's frame (315/2)
double previous_error = 0;  // For derivative calculation
double integral = 0;        // For integral term
unsigned long previous_time = 0;  // For time step calculation

// Motor base speed (adjust based on your robot's desired speed)
int base_speed = 75;
bool go = false;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  Serial.println("Serial is starting");
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW); // Keep fan off
  pinMode(START_SIG, INPUT_PULLDOWN);
  myServo.attach(SERVO, minPulse, maxPulse);


  // Initialize Pixy2 camera
  pixy.init();
  pixy.changeProg("color_connected_components");
  myServo.writeMicroseconds(maxPulse);
  // Record initial time
  previous_time = millis();
}

void loop() {

  // static bool lastButtonState = LOW;        
  int buttonState = digitalRead(START_SIG);
  if (buttonState) {
    while(digitalRead(START_SIG)){
      Serial.println("Button still pressed. Take finger off.");
    }
    go = !go;
  }


  if (go){
    // Calculate time step (dt) in seconds
    unsigned long current_time = millis();
    double dt = (current_time - previous_time) / 1000.0;
    previous_time = current_time;

    // Get colour features  features from Pixy
    pixy.ccc.getBlocks();

    myServo.writeMicroseconds(maxPulse);
    // Check if at least one vector (line) is detected
    if (pixy.ccc.getBlocks() > 0) {
      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 1) {
          // Use the x-position of the vector's tail (closest to robot)
          int x = pixy.ccc.blocks[0].m_x;

          // Calculate error (setpoint - current position)
          double error = setpoint - x;
          // working at 5, setting to 10 to match fan testing
          if (abs(error) < 10) { 
            error = 0; // Filter out wobble/correction for very small errors
          }

          // Update integral term
          integral += error * dt;

          // Calculate derivative term
          double derivative = (error - previous_error) / dt;

          // Compute PID output
          double output = Kp * error + Ki * integral + Kd * derivative;

          // Store current error for next iteration
          previous_error = error;

          // Calculate motor speeds (differential drive)
          int left_speed = constrain(base_speed + output, -150, 150);
          int right_speed = constrain(base_speed - output, -150, 150);

          // Apply speeds to motors
          setMotorSpeeds(left_speed, right_speed);

        } else if (pixy.ccc.blocks[i].m_signature == 2) {
          // pixy.ccc.blocks[i].print();
          // int x_range = 165 - pixy.ccc.blocks[i].m_x;
          // int y_range = pixy.ccc.blocks[i].m_y;
          // if (abs(x_range) < 50 && y_range > 45){
            myServo.writeMicroseconds(maxPulse);
            // brake();
            // go = false;
            Serial.println("Bullseye Detected in range");
          // } else {
          //   Serial.println("No bullseye in stopping range");
          // }
        }
      }
    } else {
      // No line detected; stop the robot
      setMotorSpeeds(0, 0);
      Serial.println("No line detected");
    }
  }
  else {
    // If off, turn motors off (brake)
    setMotorSpeeds(0, 0);
  }
}

// Function to set motor speeds and directions
void setMotorSpeeds(int left_speed, int right_speed) {
  // Control left motor
  if (left_speed > 0) {
    analogWrite(u2_IN1, LOW);
    analogWrite(u2_IN2, left_speed);
  } else if (left_speed < 0) {
    analogWrite(u2_IN1, -left_speed); // Convert negative speed to positive PWM
    analogWrite(u2_IN2, LOW);
  } else {
    analogWrite(u2_IN1, HIGH);
    analogWrite(u2_IN2, HIGH);
  }

  // Control right motor
  if (right_speed > 0) {
    analogWrite(u3_IN1, right_speed);
    analogWrite(u3_IN2, LOW);
  } else if (right_speed < 0) { 
    analogWrite(u3_IN1, LOW);
    analogWrite(u3_IN2, -right_speed); // Convert negative speed to positive PWM
  } else {
    analogWrite(u3_IN1, HIGH);
    analogWrite(u3_IN2, HIGH); 
  }
}

void brake() {
  setMotorSpeeds(-60, -60);
  delay(200);
  setMotorSpeeds(0, 0);
}

