#pragma once // “Include this file only once when compiling, even if it’s included multiple times elsewhere.”

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
const int CALIBRATION_PWM = 80;  // Tune based on your motor response
const int CALIBRATION_TIME = 10000; // ms