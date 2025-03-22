#pragma once
#include <Arduino.h>

class Motor {
private:
    int in1Pin;
    int in2Pin;
    bool reversed;
    int currentSpeed;  // Stores the last set speed

public:
    // Constructor: specify IN1, IN2 pins and if the motor is reversed
    Motor(int in1, int in2, bool isReversed = false);

    // Set motor speed (-255 to 255)
    void setSpeed(int speed);

    // Get current speed
    int getSpeed() const;

    void stop();
};