#include "Config.h"
#include "PIDController.h"

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), integral(0), previousError(0), lastTime(millis()) {}

double PIDController::compute(float error) { // does changing it to double change anything?
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Try this??? ---> pixy.ccc.getBlocks();
    double derivative = (error - previousError) / dt;
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