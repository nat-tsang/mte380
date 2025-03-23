#include <Pixy2.h>
#include <Encoder.h>

// Pixy2 Object
Pixy2 pixy;

// Motor & Encoder Pins
const int ENCODER_IN3 = 16; // Left encoder
const int ENCODER_IN4 = 17;
const int ENCODER_IN5 = 14; // Right encoder
const int ENCODER_IN6 = 21;

const int u2_IN2 = 8;  // Right motor
const int u2_IN1 = 9;  
const int u3_IN2 = 6;  // Left motor
const int u3_IN1 = 7;  

const int START_SIG = 22; // Start button

// Encoder objects
Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6); // Right
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4); // Left

// PID Constants (Tune these values)
double Kp_line = 0.5;   // Line following proportional gain
double Ki_line = 0.0;   // Integral gain (may not be needed)
double Kd_line = 0.08;  // Derivative gain

double Kp_speed = 0.016; // Speed control proportional gain

// Target speed in encoder ticks per second
float base_speed = 1600; // Adjust as needed
float motor_calibration = 1.0045; // Adjust after testing

// PID Variables
double setpoint = 157.5; // Center of Pixy2 frame (315/2)
double previous_error = 0;
double integral = 0;
unsigned long previous_time = 0;

bool go = false;

// Motor PWM values
int leftPWMOutput = 70;
int rightPWMOutput = 70;

// Previous encoder positions
long prevLeftPos = 0;
long prevRightPos = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  pinMode(START_SIG, INPUT_PULLDOWN);
  
  pixy.init();
  pixy.changeProg("color_connected_components");

  drive1Encoder.write(0);
  drive2Encoder.write(0);
  
  previous_time = millis();
  delay(5000); // Delay for setup
}

void loop() {
  unsigned long current_time = millis();
  double dt = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  int buttonState = digitalRead(START_SIG);
  if (buttonState) {
      while(digitalRead(START_SIG)){
          Serial.println("Button still pressed. Take finger off.");
      }
      go = !go;
  }

  if (go) {
      // Get color features from Pixy
      pixy.ccc.getBlocks();

      if (pixy.ccc.numBlocks > 0) {
          for (int i = 0; i < pixy.ccc.numBlocks; i++) {
              if (pixy.ccc.blocks[i].m_signature == 1) {
                  int x = pixy.ccc.blocks[i].m_x;
                  double error = setpoint - x;
                  if (abs(error) < 5) { error = 0; } // Deadzone

                  // PID calculations for line following
                  integral += error * dt;
                  double derivative = (error - previous_error) / dt;
                  double output = Kp_line * error + Ki_line * integral + Kd_line * derivative;
                  previous_error = error;

                  // Adjust target speeds based on PID output
                  float left_target_speed = base_speed + output;
                  float right_target_speed = base_speed - output;

                  // Speed control loop
                  updateMotorSpeeds(left_target_speed, right_target_speed);
              } 
              else if (pixy.ccc.blocks[i].m_signature == 2) {
                  if (pixy.ccc.blocks[i].m_width > 30 && pixy.ccc.blocks[i].m_height > 64) {
                      setMotorSpeeds(0, 0);
                      go = false;
                      Serial.println("Bullseye Detected in range");
                  } else {
                      Serial.println("No bullseye in stopping range");
                  }
              }
          }
      } else {
          setMotorSpeeds(0, 0);
          Serial.println("No line detected");
      }
  } else {
      setMotorSpeeds(0, 0);
  }
}

void updateMotorSpeeds(float left_target_speed, float right_target_speed) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;

    if (deltaTime >= 0.01) { 
        long currentLeftPos = drive2Encoder.read();
        long currentRightPos = drive1Encoder.read();

        float left_speed = (currentLeftPos - prevLeftPos) / deltaTime;
        float right_speed = (currentRightPos - prevRightPos) / deltaTime;

        float left_error = left_target_speed - left_speed;
        float right_error = right_target_speed - right_speed;

        leftPWMOutput += Kp_speed * left_error;
        rightPWMOutput += Kp_speed * right_error;

        leftPWMOutput = constrain(leftPWMOutput, 0, 240);
        rightPWMOutput = constrain(rightPWMOutput, 0, 240);

        setMotorSpeeds(leftPWMOutput, rightPWMOutput);

        Serial.print("L error: "); Serial.print(left_error);
        Serial.print(" | R error: "); Serial.print(right_error);
        Serial.print(" | Left Speed: "); Serial.print(left_speed);
        Serial.print(" | Right Speed: "); Serial.print(right_speed);
        Serial.print(" | Left PWM: "); Serial.print(leftPWMOutput);
        Serial.print(" | Right PWM: "); Serial.println(rightPWMOutput);

        prevLeftPos = currentLeftPos;
        prevRightPos = currentRightPos;
        lastTime = currentTime;
    }
}

void setMotorSpeeds(int left_speed, int right_speed) {
    if (left_speed > 0) {
        analogWrite(u3_IN1, left_speed);
        analogWrite(u3_IN2, LOW);
    } else {
        analogWrite(u3_IN1, LOW);
        analogWrite(u3_IN2, -left_speed);
    }

    if (right_speed > 0) {
        analogWrite(u2_IN1, LOW);
        analogWrite(u2_IN2, right_speed);
    } else {
        analogWrite(u2_IN1, -right_speed);
        analogWrite(u2_IN2, LOW);
    }
}
