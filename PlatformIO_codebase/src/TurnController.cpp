#include "TurnController.h"

TurnController::TurnController(Motor& leftM, Motor& rightM, EncoderReader& leftE, EncoderReader& rightE, float wheelBaseMeters, float wheelDiameterMeters)
    : leftMotor(leftM), rightMotor(rightM), leftEncoder(leftE), rightEncoder(rightE), wheelBase(wheelBaseMeters) {
    wheelCircumference = PI * WHEEL_DIAMETER;
}

void TurnController::turnDegrees(float degrees, int turnSpeed) {
    // Reset encoders
    leftEncoder.reset();
    rightEncoder.reset();

    // Calculate the arc length each wheel needs to travel
    float turnCircumference = PI * WHEEL_BASE;
    float arcLength = (abs(degrees) / 360.0) * turnCircumference;  // meters

    // Calculate equivalent encoder ticks for this arc length
    float metersPerTick = wheelCircumference / COUNTS_PER_WHEEL_REV;
    long targetTicks = arcLength / metersPerTick;

    // Determine turn direction
    int leftSpeed = degrees > 0 ? turnSpeed : -turnSpeed;
    int rightSpeed = -leftSpeed;

    // Start turning
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    // Monitor encoder ticks until target reached
    while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
        // Optionally print progress
        Serial.print("Left ticks: "); Serial.print(abs(leftEncoder.getTicks()));
        Serial.print(" | Right ticks: "); Serial.println(abs(rightEncoder.getTicks()));
        delay(10);
    }

    // Stop motors after turn
    leftMotor.stop();
    rightMotor.stop();
}