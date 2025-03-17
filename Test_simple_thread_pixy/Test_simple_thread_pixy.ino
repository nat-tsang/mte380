#include <TeensyThreads.h>
#include <Pixy2.h>
#include <Encoder.h>

Pixy2 pixy;

const int LEFT_IN2 = 6;   // left wheel
const int LEFT_IN1 = 7;   // left wheel
const int RIGHT_IN2 = 8;  // right wheel
const int RIGHT_IN1 = 9;  // right wheel

const int ENCODER_IN3 = 16;  // Right
const int ENCODER_IN4 = 17;  // Right
const int ENCODER_IN5 = 18;  // Left
const int ENCODER_IN6 = 19;  // Left

const int FAN_CTRL = 20;
const int START_SIG = 22;  // Pin 22 is connected to button

const int SPEED = 80;
bool buttonPressed = false;
bool objectDetected = false;

/* Global Variables for 180 Degree Turn */
int diameter = 8;  // Distance from wheel to wheel (measurement taken from outside of wheel)
int radius = diameter / 2;
float circumfrence = 2 * PI * radius;
float turn_distance = circumfrence / 2;
float wheelRadius = 1;
float wheelCirc = 2 * PI * wheelRadius;

Encoder rightEncoder(ENCODER_IN3, ENCODER_IN4);  // This will also set the pins as INPUT
Encoder leftEncoder(ENCODER_IN5, ENCODER_IN6);

void controlThread() {
  while (true) {
    if (objectDetected) {
      Serial.println("green object detected");
      // make piddy reverse
      reverse(SPEED);
      delay(1000);  // Run for 2 sec
    }
    threads.yield();
  }
}

void lineTrackingThread() {
  pixy.changeProg("line_tracking");
  while (true) {
    pixy.line.getMainFeatures();
    Serial.println(pixy.line.numVectors);
    if (pixy.line.numVectors == 1 && !objectDetected) {
      moveFwd(SPEED);
      delay(1000);
    } else {
      brake();
    }
    delay(100);
    threads.yield();
  }
}

void colourClassThread() {
  pixy.changeProg("color_connected_components");
  while (true) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == 1) {
          objectDetected = true;
          break;
        }
      }
    }
    delay(100);
    threads.yield();
  }
}

void turn(bool direction, int speed) {
  if (direction) {
    // RIGHT is TRUE
    analogWrite(LEFT_IN1, LOW);
    analogWrite(LEFT_IN2, speed);
    analogWrite(RIGHT_IN1, LOW);
    analogWrite(RIGHT_IN2, speed);
  } else {
    // LEFT is FALSE
    analogWrite(LEFT_IN1, speed);
    analogWrite(LEFT_IN2, LOW);
    analogWrite(RIGHT_IN1, speed);
    analogWrite(RIGHT_IN2, LOW);
  }
}

void turn180(int speed) {
  rightEncoder.write(0);
  float numRevs = (circumfrence / 2) / wheelCirc;
  float totalTicks = numRevs * 119;
  Serial.println(totalTicks);
  int currentTicks = abs(rightEncoder.read());

  delay(100);

  while (currentTicks < totalTicks) {
    Serial.print("Turning, current ticks is: ");
    Serial.println(currentTicks);
    analogWrite(RIGHT_IN1, LOW);
    analogWrite(RIGHT_IN2, speed);
    analogWrite(LEFT_IN1, LOW);
    analogWrite(LEFT_IN2, speed);
    currentTicks = abs(rightEncoder.read());
  }
}

void brake() {
  analogWrite(LEFT_IN1, HIGH);
  analogWrite(LEFT_IN2, HIGH);
  analogWrite(RIGHT_IN1, HIGH);
  analogWrite(RIGHT_IN2, HIGH);
}

void moveFwd(int speed) {  // Move Left and RIGHT wheel forward (IN1 = HIGH, IN2 = LOW)
  analogWrite(LEFT_IN1, LOW);
  analogWrite(LEFT_IN2, speed);
  analogWrite(RIGHT_IN1, speed);
  analogWrite(RIGHT_IN2, LOW);
}

void reverse(int speed) {  // Move LEFT and RIGHT wheel backward (IN1 = HIGH, IN2 = LOW)
  analogWrite(LEFT_IN1, speed);
  analogWrite(LEFT_IN2, LOW);
  analogWrite(RIGHT_IN1, LOW);
  analogWrite(RIGHT_IN2, speed);
}

void setup() {
  Serial.begin(115200);  // Start Serial Monitor
  while (!Serial)
    ;  // Wait for Serial Monitor to open
  Serial.println("Serial is starting");
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(START_SIG, INPUT_PULLDOWN);
  pinMode(FAN_CTRL, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  //LED Off

  //Pixy Initialization
  pixy.init();
  threads.addThread(controlThread);
  threads.addThread(lineTrackingThread);
  threads.addThread(colourClassThread);
}

void loop() {}