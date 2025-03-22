// #include <PID_v1.h>
// Follows tail of line
#include <Pixy2.h>

Pixy2 pixy;
const int ENCODER_IN3 = 16; // Drive 2 connected to U3
const int ENCODER_IN4 = 17; // Drive 2
const int ENCODER_IN5 = 14; // Drive 1 NEW PINS (right) connected to U2
const int ENCODER_IN6 = 21; // Drive 1 NEW PINS

const int FAN = 20;
const int u2_IN2 = 8; 
const int u2_IN1 = 9; 
const int u3_IN2 = 6;
const int u3_IN1 = 7; // left 

const int START_SIG = 22; // Pin 22 is connected to button

const int targetSpeed = 700; // Desired speed in counts per second

Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6);    // This will also set the pins as INPUT
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4);  // left

long lastLeftPos = 0;
long lastRightPos = 0;

// PID constants (tune these values based on your robot's behavior)
double Kp = 1.31;  // Proportional gain
double Ki = 0.0;  // Integral gain (may not be needed for line following)
double Kd = 0.09;  // Derivative gain (start at 0 before tuning)

// PID variables
double setpoint = 39;       // Center of Pixy's frame (79/2)
double previous_error = 0;  // For derivative calculation
double integral = 0;        // For integral term
unsigned long previous_time = 0;  // For time step calculation

// Motor base speed (adjust based on your robot's desired speed)
// int base_speed_u2 = 60;
// int base_speed_u3 = base_speed_u2 + 0;

bool go = false;
unsigned long lastTime = 0;


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

  // Initialize Pixy2 camera
  pixy.init();
  pixy.changeProg("line");
  pixy.setLamp(1, 0);

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

    measureSpeed();
    // Get line features from Pixy
    pixy.line.getMainFeatures();

    // Check if at least one vector (line) is detected
    if (pixy.line.numVectors > 0) {
      // Use the x-position of the vector's tail (closest to robot)
      int x_head = pixy.line.vectors[0].m_x1;
      int x_tail = pixy.line.vectors[0].m_x0;

      int x = x_tail + 0.5*(x_head - x_tail); // x at it's mid point
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

      // Calculate motor speeds (differential drive)
      int left_speed = constrain(base_speed_u3 + output, -150, 150);
      int right_speed = constrain(base_speed_u2 - output, -150, 150);

      // Apply speeds to motors
      setMotorSpeeds(left_speed, right_speed);

      // Debug output
      Serial.print("x: ");
      Serial.print(x);
      Serial.print(" error: ");
      Serial.print(error);
      Serial.print(" output: ");
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
  }
  else {
    // If off, turn motors off (brake)
    setMotorSpeeds(0,0);
  }
}


void determinePWM() {
  long currentLeftPos = drive2Encoder.read();
  long currentRightPos = drive1Encoder.read();
  unsigned long currentTime = millis();

  long leftChange = currentLeftPos - lastLeftPos;
  long rightChange = currentRightPos - lastRightPos;
  unsigned long timeChange = currentTime - lastTime;

  if (timeChange > 0) { // Prevent divide by zero
    float leftSpeedCPS = (leftChange / (float)timeChange) * 1000.0;  // Counts per second
    float rightSpeedCPS = (rightChange / (float)timeChange) * 1000.0;

    if (leftSpeedCPS || rightSpeedCPS){   // Turn LED HIGH if wheels are spinning
      digitalWrite(LED_BUILTIN, HIGH);
    }
    Serial.print("PWM: ");
    Serial.print(pwmValue);
    Serial.print(" | Left Speed: ");
    Serial.print(leftSpeedCPS);
    Serial.print(" cps | Right Speed: ");
    Serial.print(rightSpeedCPS);
    Serial.println(" cps");
  }

  // Update last values for next iteration
  lastLeftPos = currentLeftPos;
  lastRightPos = currentRightPos;
  lastTime = currentTime;
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

