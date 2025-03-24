#pragma once
#include <Arduino.h>
#include "Motor.h"

// Global robot run state
extern bool robotRunning;

// Initialize button pin
void initButton(int pin);

// Check button state and toggle robot running state
void checkButton(Motor& leftMotor, Motor& rightMotor);


// class Helpers {
//     private:
//         int buttonState;
//         bool go = false; 

//     public:
//         Helpers();

//         void buttonCheck();
// };