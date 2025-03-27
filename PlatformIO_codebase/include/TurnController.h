#pragma once
#include <Arduino.h>
#include <Motor.h>
#include <EncoderReader.h>
#include <PixyLineTracker.h>

class TurnController {
private:
    Motor& leftMotor;
    Motor& rightMotor;
    EncoderReader& leftEncoder;
    EncoderReader& rightEncoder;
    float wheelBase;         // Distance between wheels in meters
    float wheelCircumference;

public:
    TurnController(Motor& leftM, Motor& rightM, EncoderReader& leftE, EncoderReader& rightE, float wheelBaseMeters, float wheelDiameterMeters);

    // Perform a turn (positive angle = right turn, negative = left turn)
    void turnDegrees(float degrees, int turnSpeed);
    void turnToRedLine(int turnSpeed, const Block* block, int numBlock);
};