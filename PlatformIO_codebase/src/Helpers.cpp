#include "Helpers.h"

bool robotRunning = false;     // Global state flag
static int buttonPin;
static bool lastButtonState = LOW;  // Default LOW due to external pull-down

void initButton(int pin) {
    buttonPin = pin;
    pinMode(buttonPin, INPUT_PULLDOWN);
}

void checkButton(Motor& leftMotor, Motor& rightMotor) {
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


// Helpers::Helpers() {}

// void Helpers::buttonCheck()
// {
//     buttonState = digitalRead(START_SIG);
//     if (buttonState) {
//         while (digitalRead(START_SIG)) {
//             Serial.println("Button still pressed. Take finger off.");
//         }
//         go = !go;
//     }
// }