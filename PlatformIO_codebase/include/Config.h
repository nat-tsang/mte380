#pragma once // “Include this file only once when compiling, even if it’s included multiple times elsewhere.”
#include <Arduino.h>

// Motor pins
const int u2_IN2 = 8;  // Right motor
const int u2_IN1 = 9;  
const int u3_IN2 = 6;  // Left motor
const int u3_IN1 = 7;  

// Encoder Pins
// Moving fwd will increase encoder count
const int ENCODER_IN3 = 16; // Drive 2 connected to U3
const int ENCODER_IN4 = 17; // Drive 2
const int ENCODER_IN5 = 14; // Drive 1 NEW PINS (right) connected to U2
const int ENCODER_IN6 = 21; // Drive 1 NEW PINS

const int START_SIG = 22; // Pin 22 is connected to button

const int SERVO = 5;
const int minPulse = 700;   // open
const int maxPulse = 2000;  // closed

const int FAN = 20;

// Calibration settings
const int CALIBRATION_PWM = 65;  // Tune based on your motor response
const int CALIBRATION_TIME = 10000; // ms

// PID Gains for Line Following
const float LINE_KP = 0.5;     // Proportional gain
const float LINE_KI = 0.0;     // Integral gain (start with 0, add if necessary)
const float LINE_KD = 0.08;     // Derivative gainconst

const float WHEEL_BASE = 0.08; // Distance between wheels in meters
const float WHEEL_DIAMETER = 0.021; // Wheel diameter in meters
const float COUNTS_PER_WHEEL_REV = 120.0;  // Real-world counts per wheel rev
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // meters
const float LEGO_KPx = 0.5;
const float LEGO_KPy = 0.3;

// Pixy CCC params
const float X_CENTER = 315/2;
const float Y_CENTER = 207/2;
const float LEGO_SIG = 4;
const float BULLSEYE_SIG = 2;
const float REDLINE_SIG = 1;