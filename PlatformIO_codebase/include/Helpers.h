#pragma once
#include <Arduino.h>
#include "Motor.h"

// #define BTSerial Serial1

// Global robot run state
extern bool robotRunning;
extern bool SYSTEM_SHUTDOWN;

// Initialize button pin
void initButton(int pin);

// Check button state and toggle robot running state
void checkButton(Motor& leftMotor, Motor& rightMotor);

// void debugPrint(String msg);
