#include <Encoder.h>

const int ENCODER_IN3 = 16; // Drive 2 connected to U3
const int ENCODER_IN4 = 17; // Drive 2
const int ENCODER_IN5 = 14; // Drive 1 NEW PINS (right) connected to U2
const int ENCODER_IN6 = 21; // Drive 1 NEW PINS
const int FAN = 20;
const int u2_IN2 = 8; //right
const int u2_IN1 = 9; 
const int u3_IN2 = 6;
const int u3_IN1 = 7; // left 

// const int START_SIG = 22; // Pin 22 is connected to button

Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6);    // This will also set the pins as INPUT
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4);  // left


// Speed control parameters
float Kp = 0.016;                 // Proportional gain for speed control
float motor_calibration = 1.0045;  // Adjust after testing
float right_target_speed = 1600; 
// float left_target_speed = right_target_speed * motor_calibration;;   // ticks per second
float left_target_speed = right_target_speed;;   // ticks per second


// Control loop state
long prevLeftPos = 0;
long prevRightPos = 0;
int leftPWMOutput = 40;  // Start at some base PWM
int rightPWMOutput = 40;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  drive1Encoder.write(0);
  drive2Encoder.write(0);
  delay (7000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // seconds

  if (deltaTime >= 0.01) { 
    long currentLeftPos = drive2Encoder.read();
    long currentRightPos = drive1Encoder.read();

    // Calculate speed in ticks per second
    float left_speed = (currentLeftPos - prevLeftPos) / deltaTime;
    float right_speed = (currentRightPos - prevRightPos) / deltaTime;

    // Calculate error
    float adjustment = left_target_speed - left_speed;
    float left_error = left_target_speed - left_speed;
    float right_error = right_target_speed - right_speed;

    leftPWMOutput += Kp * left_error;
    rightPWMOutput += Kp * right_error;

    // Constrain PWM values
    leftPWMOutput = constrain(leftPWMOutput, 0, 240);
    rightPWMOutput = constrain(rightPWMOutput, 0, 240);

    // Apply PWM
    setMotorSpeeds(leftPWMOutput, rightPWMOutput);

    // Debugging
    Serial.print("L error: "); Serial.print(left_error);
    Serial.print(" | R error: "); Serial.print(right_error);
    Serial.print(" | Left Speed: "); Serial.print(left_speed);
    Serial.print(" | Right Speed: "); Serial.print(right_speed);
    Serial.print(" | Left PWM: "); Serial.print(leftPWMOutput);
    Serial.print(" | Right PWM: "); Serial.print(rightPWMOutput);
    Serial.print(" | Adjustment: "); Serial.println(adjustment);

    // Update previous values
    prevLeftPos = currentLeftPos;
    prevRightPos = currentRightPos;
    lastTime = currentTime;
  }
}

// Function to set motor speeds and directions
void setMotorSpeeds(int left_speed, int right_speed) {
  // Control left motor
  analogWrite(u3_IN1, left_speed);
  analogWrite(u3_IN2, LOW);
  // // Control right motor
  analogWrite(u2_IN1, LOW);
  analogWrite(u2_IN2, right_speed);
}