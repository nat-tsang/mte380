#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <Arduino.h>
#include <Encoder.h>

// Adjustable parameters
extern float Kp;
extern float motor_calibration;
extern float right_target_speed;
extern float left_target_speed;

// Function declaration
void closedLoopVelocityControl(Encoder &leftEncoder, Encoder &rightEncoder, int &leftPWM, int &rightPWM, 
                               int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2, 
                               long &prevLeftPos, long &prevRightPos, unsigned long &lastTime);

// Utility
void setMotorSpeeds(int left_speed, int right_speed, int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2);

#endif