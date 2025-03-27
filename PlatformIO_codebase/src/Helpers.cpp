#include "Helpers.h"

bool robotRunning = false;     // Global state flag
bool SYSTEM_SHUTDOWN = false;  // Set to true to enable shutdown
static int buttonPin;
static bool lastButtonState = LOW;  // Default LOW due to external pull-down

void initButton(int pin) {
    buttonPin = pin;
    pinMode(buttonPin, INPUT_PULLDOWN);
}

void checkButton(Motor& leftMotor, Motor& rightMotor) {
    if (SYSTEM_SHUTDOWN) return;
    
    bool buttonState = digitalRead(buttonPin);

    if (lastButtonState == HIGH && buttonState == LOW) {
        // Button press detected (falling edge)
        robotRunning = !robotRunning;
        Serial.print("Robot is now ");
        Serial.println(robotRunning ? "RUNNING" : "STOPPED");

        if (!robotRunning) {
            leftMotor.stop();
            rightMotor.stop();
        }
    }
    lastButtonState = buttonState;
}