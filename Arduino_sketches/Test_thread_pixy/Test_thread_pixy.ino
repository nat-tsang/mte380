#include <TeensyThreads.h>
#include <Pixy2.h>

Pixy2 pixy;
// Motor + Board Parameters
const int LEFT_IN2 = 6;
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

// Global Variables for State Transitions
volatile bool buttonPressed = false;
volatile bool greenDetected = false;
volatile bool goingOut = true;
volatile bool legoManCaptured = false;
volatile bool bullseyeDetected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("serial is starting");
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
}

void controlThread() {
  while (true) {
    int buttonState = digitalRead(START_SIG);

    if (buttonState) {
      while (digitalRead(START_SIG)) {
        Serial.println("Button still pressed. Take finger off.");
      }

      if (bullseyeDetected && !legoManCaptured) {
        Serial.println("detected bullseye, finding lego man")
        legoManCaptured = findLegoMan(); //subroutine for collecting legoman, returns true
      }

      if (legoManCaptured && goingOut) {
        Serial.println("legoman captured, turning 180");
        execute180Turn();
        goingOut = false; // marks that PiDdy is no longer going there, now returning
        bullseyeDetected = false; // Reset
      }

      if (!goingOut && greenDetected) {
        Serial.println("detected green box, dropping off legoman");
        releaseLegoMan();
        greenDetected = false;
        legoManCaptured = false; 
      }
      threads.yield();
    }
  }
}

void lineTrackingThread() {
  pixy
  while (true) {
    if (!greenDetected && !bullseyeDetected) {
      
    }

    // Green box detected within specific threshold
    if (!goingOut && greenObjectDetected) {
      Serial.println("green object detected, switching to deposit lego man");
      delay(100);  // delay to prevent busy-waiting (this is where a cpu constantly checks a condition instead of running other important tasks)
      threads.yield();
    } else {
      pixy.line.getMainFeatures();
      Serial.println("tracking line...");
      delay(50);
      // Main Line tracking code (insert member functions from PD controller)
    }
  }
}

void colourClassification() {
  while (true) {
    pixy.ccc.getBlocks();

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      // Pixy should only be trained on Lego Man colour (1) + green box (2)?
      if (pixy.ccc.blocks[i].signature == 2) {
        greenObjectDetected = true;
        Serial.println("green object detected, switching ")
      }
    }

    if (greenObjectDetected && !legoManDropped) {
      // enter dropping Lego man code then yield this thread to line tracking
    }
  }
}