#include "Helpers.h"

bool robotRunning = false;     // Global state flag
bool SYSTEM_SHUTDOWN = false;  // Set to true to enable shutdown
static int buttonPin;
static bool lastButtonState = LOW;  // Default LOW due to external pull-down
int rightbasePWM = 66;
int leftbasePWM = 66;

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

String getTimestamp() {
    unsigned long ms = millis();
    unsigned long totalSeconds = ms / 1000;
    unsigned int hours = totalSeconds / 3600;
    unsigned int minutes = (totalSeconds % 3600) / 60;
    unsigned int seconds = totalSeconds % 60;
  
    char buffer[12];  // HH:MM:SS + null terminator
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", hours, minutes, seconds);
    return String(buffer);
  }

void logWithTimestamp() {
    BTSerial.print("[");
    BTSerial.print(getTimestamp());
    BTSerial.print("] ");
}

void debugPrint(String msg) {
    // logWithTimestamp();
    Serial.println(msg);
    BTSerial.println(msg);
}