#include "TurnController.h"
#include "PixyLineTracker.h"
#include "Config.h"
#include "Helpers.h"

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
    int rightSpeed = -(leftSpeed+4);

    // Start turning
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    // Monitor encoder ticks until target reached
    while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
        // Optionally print progress
        BTSerial.print("Left ticks: "); BTSerial.print(abs(leftEncoder.getTicks()));
        BTSerial.print(" | Right ticks: "); BTSerial.print(abs(rightEncoder.getTicks()));
        BTSerial.print(" | Target ticks: "); BTSerial.println(targetTicks);
        delay(10);
    }

    // Stop motors after turn
    leftMotor.stop();
    rightMotor.stop();
}

void TurnController::turnToShayla(int turnSpeed, const Block* block, int numBlock)
{
    float turnCircumference = PI * WHEEL_BASE;
    float arcLength = (20.0 / 360.0) * turnCircumference;
    float metersPerTick = wheelCircumference / COUNTS_PER_WHEEL_REV;
    long targetTicks = arcLength / metersPerTick;

    int spinLimit = 0;
    bool foundShayla = false;

    while (!foundShayla && spinLimit < (360/20)) {
        leftEncoder.reset();
        rightEncoder.reset();

        // Determine turn direction
        int leftSpeed = turnSpeed;
        int rightSpeed = -(turnSpeed+4);

        leftMotor.setSpeed(leftSpeed);
        rightMotor.setSpeed(rightSpeed);

        while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
            BTSerial.print("Left ticks: "); BTSerial.print(abs(leftEncoder.getTicks()));
            BTSerial.print(" | Right ticks: "); BTSerial.println(abs(rightEncoder.getTicks()));
            delay(10);
        }

        leftMotor.stop();
        rightMotor.stop();

        if (numBlock > 0) {
            for (int i = 0; i < numBlock; i++) {
                if (block[i].m_signature == LEGO_SIG) {
                    debugPrint("Found shayla.");
                    foundShayla = true;
                    break;
                }
            }
        }

        if (!foundShayla) {
            debugPrint("Still can't find Shayla.");
            spinLimit++;
            if (spinLimit >= (360/20)) {
                debugPrint("Spun 360 degrees and no Shayla found. Exiting.");
            }
        }

        delay(200); // Optional pause before next spin
    }
}

// void repositionToShay(int turnSpeed, int degrees, int drivr) {
//     // Reset encoders
//     leftEncoder.reset();
//     rightEncoder.reset();

//     // Calculate the arc length each wheel needs to travel
//     float turnCircumference = PI * WHEEL_BASE;
//     float arcLength = (abs(degrees) / 360.0) * turnCircumference;  // meters

//     // Calculate equivalent encoder ticks for this arc length
//     float metersPerTick = wheelCircumference / COUNTS_PER_WHEEL_REV;
//     long targetTicks = arcLength / metersPerTick;

//     // Determine turn direction
//     int leftSpeed = degrees > 0 ? turnSpeed : -turnSpeed;
//     int rightSpeed = -(leftSpeed+4);

//     // Start turning
//     leftMotor.setSpeed(leftSpeed);
//     rightMotor.setSpeed(rightSpeed);

//     // Monitor encoder ticks until target reached
//     while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
//         // Optionally print progress
//         BTSerial.print("Left ticks: "); BTSerial.print(abs(leftEncoder.getTicks()));
//         BTSerial.print(" | Right ticks: "); BTSerial.print(abs(rightEncoder.getTicks()));
//         BTSerial.print(" | Target ticks: "); BTSerial.println(targetTicks);
//         delay(10);
//     }

//     // Stop motors after turn
//     leftMotor.stop();
//     rightMotor.stop();    
// }

// void TurnController::turnToLegoMan(int turnSpeed, const Block* block, int numBlock)
// {    float turnCircumference = PI * WHEEL_BASE;
//     float arcLength = (30.0 / 360.0) * turnCircumference;
//     float metersPerTick = wheelCircumference / COUNTS_PER_WHEEL_REV;
//     long targetTicks = arcLength / metersPerTick;

//     int spinLimit = 0;
//     bool foundRedLine = false;

//     while (!foundRedLine && spinLimit < 12) {
//         leftEncoder.reset();
//         rightEncoder.reset();

//         // Determine turn direction
//         int leftSpeed = turnSpeed;
//         int rightSpeed = -turnSpeed;

//         leftMotor.setSpeed(leftSpeed);
//         rightMotor.setSpeed(rightSpeed);

//         while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
//             BTSerial.print("Left ticks: "); BTSerial.print(abs(leftEncoder.getTicks()));
//             BTSerial.print(" | Right ticks: "); BTSerial.println(abs(rightEncoder.getTicks()));
//             delay(10);
//         }

//         leftMotor.stop();
//         rightMotor.stop();

//         if (numBlock > 0) {
//             for (int i = 0; i < numBlock; i++) {
//                 if (block[i].m_signature == REDLINE_SIG) {
//                     debugPrint("Red line detected.");
//                     foundRedLine = true;
//                     break;
//                 }
//             }
//         }

//         if (!foundRedLine) {
//             debugPrint("No red line detected.");
//             spinLimit++;
//             if (spinLimit >= 12) {
//                 debugPrint("Spun 360 degrees and no red line found. Exiting.");
//             }
//         }

//         delay(200); // Optional pause before next spin
//     }
// }
