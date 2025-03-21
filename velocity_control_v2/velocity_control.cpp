#include "velocity_control.h"

// Global gains and targets
float Kp = 0.016;
float motor_calibration = 1.0045;
float right_target_speed = 1600;
float left_target_speed = right_target_speed; 

void closedLoopVelocityControl(Encoder &leftEncoder, Encoder &rightEncoder, int &leftPWM, int &rightPWM, 
                               int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2, 
                               long &prevLeftPos, long &prevRightPos, unsigned long &lastTime) {

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // seconds

    if (deltaTime >= 0.01) {
        long currentLeftPos = leftEncoder.read();
        long currentRightPos = rightEncoder.read();

        float left_speed = (currentLeftPos - prevLeftPos) / deltaTime;
        float right_speed = (currentRightPos - prevRightPos) / deltaTime;

        float left_error = left_target_speed - left_speed;
        float right_error = right_target_speed - right_speed;

        leftPWM += Kp * left_error;
        rightPWM += Kp * right_error;

        leftPWM = constrain(leftPWM, 0, 240);
        rightPWM = constrain(rightPWM, 0, 240);

        setMotorSpeeds(leftPWM, rightPWM, leftMotorPin1, leftMotorPin2, rightMotorPin1, rightMotorPin2);

        // Debugging
        Serial.print("L error: "); Serial.print(left_error);
        Serial.print(" | R error: "); Serial.print(right_error);
        Serial.print(" | L Speed: "); Serial.print(left_speed);
        Serial.print(" | R Speed: "); Serial.print(right_speed);
        Serial.print(" | L PWM: "); Serial.print(leftPWM);
        Serial.print(" | R PWM: "); Serial.println(rightPWM);

        prevLeftPos = currentLeftPos;
        prevRightPos = currentRightPos;
        lastTime = currentTime;
    }
}

void setMotorSpeeds(int left_speed, int right_speed, int leftMotorPin1, int leftMotorPin2, int rightMotorPin1, int rightMotorPin2) {
    analogWrite(leftMotorPin1, left_speed);
    analogWrite(leftMotorPin2, LOW);
    analogWrite(rightMotorPin1, LOW);
    analogWrite(rightMotorPin2, right_speed);
}