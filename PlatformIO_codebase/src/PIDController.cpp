#include "Config.h"
#include "PIDController.h"

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), integral(0), previousError(0), lastTime(millis()) {}

float PIDController::compute(float error) {
    unsigned long now = micros();
    float dt = (now - lastTime) / 1e6; // convert ms to seconds
    lastTime = now;

    if (dt <= 0) dt = 0.001;  // Prevent divide by zero
    
    if (!isnan(error) && !isinf(error) && !isnan(dt) && !isinf(dt)) {
        integral += error * dt;
    }
    float derivative = (error - previousError) / dt;
    previousError = error;

    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void PIDController::reset() {
    integral = 0;
    previousError = 0;
    lastTime = millis();
}

void PIDController::setGains(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}