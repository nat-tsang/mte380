#include "TurnController.h"
#include "PixyLineTracker.h"
#include "Config.h"

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
        Serial.print(" | Target ticks: "); Serial.println(targetTicks);
        delay(10);
    }

    // Stop motors after turn
    leftMotor.stop();
    rightMotor.stop();
}

// void TurnController::turnToLegoMan(int turnSpeed, PixyLineTracker& pixyTracker)
// {
//     leftEncoder.reset();
//     rightEncoder.reset();

//     float turnCircumference = PI * WHEEL_BASE;
//     float arcLength = (360.0 / 360.0) * turnCircumference;  // meters of a full circle (360 turn)

//     // Calculate equivalent encoder ticks for this arc length
//     float metersPerTick = wheelCircumference / COUNTS_PER_WHEEL_REV;
//     long targetTicks = arcLength / metersPerTick;

//     // Determine turn direction
//     int leftSpeed = 360.0 > 0 ? turnSpeed : -turnSpeed;
//     int rightSpeed = -leftSpeed;

//     // Start turning
//     leftMotor.setSpeed(leftSpeed);
//     rightMotor.setSpeed(rightSpeed);

//     while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
//         // Check for LegoMan during turn
//         auto [x, y] = pixyTracker.getPixyCoord(LEGO_SIG);  // signature of LEGO man
//         if (x > 0 && y > 0) {  // or use != -1 if you return -1 on failure
//             Serial.println("LEGO man found! Stopping turn.");
//             break;
//         }
//         // Optionally print progress
//         Serial.print("Left ticks: "); Serial.print(abs(leftEncoder.getTicks()));
//         Serial.print(" | Right ticks: "); Serial.println(abs(rightEncoder.getTicks()));
//         delay(10);
//     }

//     // Stop motors after turn
//     leftMotor.stop();
//     rightMotor.stop();
// }
