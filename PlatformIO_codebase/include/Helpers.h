#pragma once
#include <Arduino.h>
#include "Motor.h"

// #define BTSerial Serial1

// Global robot run state
extern bool robotRunning;

// Initialize button pin
void initButton(int pin);

// Check button state and toggle robot running state
void checkButton(Motor& leftMotor, Motor& rightMotor);

// void debugPrint(String msg);

// class Helpers {
//     private:
//         int buttonState;
//         bool go = false; 

//     public:
//         Helpers();

//         void buttonCheck();
// };