// #include <PID_v1.h>
#include <Pixy2.h>

Pixy2 pixy;
const int FAN = 20;
const int LEFT_IN2 = 8; // left wheel
const int LEFT_IN1 = 9; // left wheel
const int RIGHT_IN2 = 6; // right wheel
const int RIGHT_IN1 = 7; // right wheel
// DRV8871 Motor Control Code
const int u2_IN2 = 8; 
const int u2_IN1 = 9; 
const int u3_IN2 = 6;
const int u3_IN1 = 7; // left 

// PID constants (tune these values based on your robot's behavior)
double Ku = 1;
double Tu = 0.6466667;
double Td = 0.125*Tu;

double Kp = 0.05;  // Proportional gain    //
double Ki = 0.0;  // Integral gain (may not be needed for line following)
double Kd = 0.00;
// double Kd = Kp*Td;  // Derivative gain (start at 0 before tuning)


// PID variables
double setpoint = 39;       // Center of Pixy's frame (79/2)
double previous_error = 0;  // For derivative calculation
double integral = 0;        // For integral term
unsigned long previous_time = 0;  // For time step calculation

// Motor base speed (adjust based on your robot's desired speed)
// int base_speed = 80;
int base = 70;


void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  Serial.println("Serial is starting");
  Serial.print("Kd: ");
  Serial.println(Kd);

  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);

  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW); // Keep fan off
  // Initialize Pixy2 camera
  pixy.init();
  pixy.changeProg("line");
  pixy.setLamp(0, 1);

  // Record initial time
  previous_time = millis();
}

void loop() {
  // Calculate time step (dt) in seconds
  unsigned long current_time = millis();
  double dt = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  // Get line features from Pixy
  pixy.line.getMainFeatures();

  // Check if at least one vector (line) is detected
  if (pixy.line.numVectors > 0) {
    // Use the x-position of the vector's tail (closest to robot)
    int x = pixy.line.vectors[0].m_x0;

    // Calculate error (setpoint - current position)
    double error = setpoint - x;

    // Update integral term
    integral += error * dt;

    // Calculate derivative term
    double derivative = (error - previous_error) / dt;

    // Compute PID output
    double output = Kp * error + Ki * integral + Kd * derivative;

    // Store current error for next iteration
    previous_error = error;
    int leftSpeed, rightSpeed;

    // Determine wheel speeds based on PID output
    if (output < 0) {
        // Turn right: left wheel forward, right wheel backward
        leftSpeed = base + (-output);
        rightSpeed = - base + output;
    } else if (output > 0) {
        // Turn left: right wheel forward, left wheel backward
        leftSpeed = -base - output;
        rightSpeed = base + output;
    } else {
        // Stop if PID output is zero
        leftSpeed = 0;
        rightSpeed = 0;
    }

    // Calculate motor speeds (differential drive)
    int left_speed = constrain(leftSpeed, -200, 200);
    int right_speed = constrain(rightSpeed, -200, 200);

    // Apply speeds to motors
    setMotorSpeeds(left_speed, right_speed);

    // Debug output
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" error: ");
    Serial.print(error);
    Serial.print("PID output: ");
    Serial.print(output);
    Serial.print(" left_speed: ");
    Serial.print(left_speed);
    Serial.print(" right_speed: ");
    Serial.println(right_speed);
  } else {
    // No line detected; stop the robot
    setMotorSpeeds(0, 0);
    Serial.println("No line detected");
  }
  Serial.print("Base speed: ");
  Serial.println(base);
}

// Function to set motor speeds and directions
void setMotorSpeeds(int left_speed, int right_speed) {  
  //Control left motor
  if (left_speed > 0) {
    analogWrite(u3_IN1, left_speed);
    analogWrite(u3_IN2, LOW);
  } else if (left_speed < 0) {
    analogWrite(u3_IN1, LOW);
    analogWrite(u3_IN2, -left_speed); // Convert negative speed to positive PWM
  } else {
    analogWrite(u3_IN1, HIGH);
    analogWrite(u3_IN2, HIGH);
  }
  // Control right motor
  if (right_speed > 0) {
    analogWrite(u2_IN1, LOW);
    analogWrite(u2_IN2, right_speed);
  } else if (right_speed < 0) { 
    analogWrite(u2_IN1, -right_speed);
    analogWrite(u2_IN2, LOW);
  } else {
    analogWrite(u2_IN1, HIGH);
    analogWrite(u2_IN2, HIGH); 
  }
}

