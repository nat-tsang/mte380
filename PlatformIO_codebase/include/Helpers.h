#pragma once
#include <Arduino.h>
#include "Motor.h"

#define BTSerial Serial1

extern int rightbasePWM;  // Base PWM value       PWM of 66 allows robot to stop straight on with bullseye
extern int leftbasePWM;  // Base PWM value

void flashPixyLight(int times);

// Global robot run state
extern bool robotRunning;
extern bool SYSTEM_SHUTDOWN;

// Initialize button pin
void initButton(int pin);

// Check button state and toggle robot running state
void checkButton(Motor& leftMotor, Motor& rightMotor);

String getTimestamp();
void logWithTimestamp();
void debugPrint(String msg);
