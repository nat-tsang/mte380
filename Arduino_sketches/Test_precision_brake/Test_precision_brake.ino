#include <Encoder.h>

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

bool go = false;
bool hasStarted = false;

// Motor PWM values
int leftPWMOutput = 70;
int rightPWMOutput = 70;

// Previous encoder positions
long prevLeftPos = 0;
long prevRightPos = 0;
unsigned long startTime = 0;
unsigned long currentTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(u2_IN1, OUTPUT);
  pinMode(u2_IN2, OUTPUT);
  pinMode(u3_IN1, OUTPUT);
  pinMode(u3_IN2, OUTPUT);
  pinMode(START_SIG, INPUT_PULLDOWN);
  pinMode(LED_BUILTIN, OUTPUT);

  drive1Encoder.write(0);
  drive2Encoder.write(0);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000); // Delay for setup
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  int buttonState = digitalRead(START_SIG);
  if (buttonState) {
      while(digitalRead(START_SIG)){
          Serial.println("Button still pressed. Take finger off.");
      }
      go = !go;
      hasStarted = false;
      digitalWrite(LED_BUILTIN, LOW);
  }
  if (go) {
    if (!hasStarted) {
      Serial.println("Starting drive forward");
      startTime = millis();
      hasStarted = true;
      Serial.println("Motors started.");
    }

    currentTime = millis();

    if (currentTime - startTime <= 3000) {
      setMotorSpeeds(leftPWMOutput, rightPWMOutput);
    } else {
      Serial.println("Applying smart brake...");
      digitalWrite(LED_BUILTIN, HIGH); 
      setMotorSpeeds(0, 0);
      // smartBrake();
      go = false;
    }
  }
}


void setMotorSpeeds(int left_speed, int right_speed) {
    if (left_speed > 0) {
      analogWrite(u3_IN1, left_speed);
      analogWrite(u3_IN2, LOW);
    } else if (left_speed < 0) {
      analogWrite(u3_IN1, LOW);
      analogWrite(u3_IN2, -left_speed);
    } else {
      analogWrite(u3_IN1, 255);
      analogWrite(u3_IN2, 255);
    }

    if (right_speed > 0) {
      analogWrite(u2_IN1, LOW);
      analogWrite(u2_IN2, right_speed);
    } else if (right_speed < 0) {
      analogWrite(u2_IN1, -right_speed);
      analogWrite(u2_IN2, LOW);
    } else {
      analogWrite(u2_IN1, 255);
      analogWrite(u2_IN2, 255);
    }
}

void smartBrake() {
  static unsigned long lastTime = 0;
  float deltaTime = (currentTime - lastTime) / 1000.0; // convert ms to seconds

  if (deltaTime == 0) deltaTime = 0.01; // avoid division by zero

  long currentLeftPos = drive2Encoder.read();
  long currentRightPos = drive1Encoder.read();

  float left_speed = (currentLeftPos - prevLeftPos) / deltaTime;  // counts/sec
  float right_speed = (currentRightPos - prevRightPos) / deltaTime;

  int maxEncoderSpeed = 9000;
  int baseLeftPWM = constrain((int)(abs(left_speed) * 70.0 / maxEncoderSpeed), 0, 65);
  int baseRightPWM = constrain((int)(abs(right_speed) * 70.0 / maxEncoderSpeed), 0, 65);

  prevLeftPos = currentLeftPos;
  prevRightPos = currentRightPos;
  lastTime = currentTime;

  int steps = 10;
  int delayTime = 50;

  for (int i = steps; i > 0; i--) {
    Serial.println("Decreasing speed.");
    int scaledPWML = baseLeftPWM * i / steps;
    Serial.print(scaledPWML);
    int scaledPWMR = baseRightPWM * i / steps;
    Serial.println(scaledPWMR);
    setMotorSpeeds(scaledPWML, scaledPWMR);
    delay(delayTime);
  }

  setMotorSpeeds(-30, -30);
  delay(100);
  setMotorSpeeds(0, 0);
}

