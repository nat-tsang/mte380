#pragma once
#include <Arduino.h>

class PIDController {
private:
    float Kp, Ki, Kd;
    float integral;
    float previousError;
    unsigned long lastTime;

public:
    PIDController(float p, float i, float d);
    float compute(float error);
    void reset();  // clear integral/prev error
    void setGains(float p, float i, float d);  // update PID gains dynamically
};