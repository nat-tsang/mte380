#include "Motor.h"


Motor::Motor(int in1, int in2, bool isReversed)
    : in1Pin(in1), in2Pin(in2), reversed(isReversed), currentSpeed(0) {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
}

void Motor::setSpeed(int speed) {
    currentSpeed = constrain(speed, -255, 255);
    // Flip speed if motor is mounted reversed
    int actualSpeed = reversed ? -currentSpeed : currentSpeed;  // If reversed if true, set actualSpeed to -currentSpeed, else set it to currentSpeed
    if (actualSpeed > 0) {
        // Forward: PWM on IN1, IN2 LOW
        analogWrite(in1Pin, actualSpeed);
        analogWrite(in2Pin, LOW);
    } else if (actualSpeed < 0) {
        // Reverse: PWM on IN2, IN1 LOW
        analogWrite(in1Pin, LOW);
        analogWrite(in2Pin, -actualSpeed);
    } else {
        // Apply active braking
        analogWrite(in1Pin, 255);   // Brake
        analogWrite(in2Pin, 255);
    }
}

int Motor::getSpeed() const {
    return currentSpeed;
}


void Motor::stop() {
    setSpeed(0);
}