// Control wheel motors and encoders
#include <Servo.h>

Servo myServo;

const int SERVO = 5;
const int minPulse = 900;   // microseconds for ~0 degrees
const int maxPulse = 2100;  // microseconds for ~120 degrees

const int u2_IN2 = 8; 
const int u2_IN1 = 9; 
const int u3_IN2 = 6;
const int u3_IN1 = 7; // left 

const int START_SIG = 22; // Pin 22 is connected to button

int base_speed = 60; // PWM applied to motors
bool go = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SERVO,OUTPUT);
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  pinMode(START_SIG, INPUT_PULLDOWN);

  //Start Button
  myServo.attach(SERVO, minPulse, maxPulse);
}

void loop() {
  int buttonState = digitalRead(START_SIG);
  if (buttonState) {
    while(digitalRead(START_SIG)){
      Serial.println("Button still pressed. Take finger off.");
    }
    myServo.writeMicroseconds(minPulse);  // Move to 0° position
    Serial.println("Servo at min position");
    
    setMotorSpeeds(base_speed, base_speed);
    delay(2000);
    setMotorSpeeds(0, 0);
    myServo.writeMicroseconds(maxPulse);  // Move to ~120° position
    Serial.println("Servo at max position");
  }
  setMotorSpeeds(0, 0);
}

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



