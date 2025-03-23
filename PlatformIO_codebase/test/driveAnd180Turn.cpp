#include <Arduino.h>
#include "../include/Config.h"
#include "../src/Motor.cpp"
#include "../src/EncoderReader.cpp"
#include "../src/TurnController.cpp"

// Example: Adjust pins and objects based on your setup
Motor leftMotor(u2_IN1, u2_IN2, true);
Motor rightMotor(u3_IN1, u3_IN2, false);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);
EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);

// 12cm wheelbase, 20mm wheel diameter
TurnController turnController(leftMotor, rightMotor, leftEncoder, rightEncoder, 0.12, 0.02);

void driveStraight(long targetTicks, int speed) {
    leftEncoder.reset();
    rightEncoder.reset();
    //TODO: Check why left and right motors are switched
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed+16);

    while (abs(leftEncoder.getTicks()) < targetTicks && abs(rightEncoder.getTicks()) < targetTicks) {
        Serial.print("L Ticks: "); Serial.print(leftEncoder.getTicks());
        Serial.print(" | R Ticks: "); Serial.println(rightEncoder.getTicks());
        delay(10);
    }

    leftMotor.stop();
    rightMotor.stop();
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    // --- Step 1: Drive forward ---
    Serial.println("Driving forward...");
    long forwardTicks = 1000;  // Adjust based on distance you want
    driveStraight(forwardTicks, 70);

    delay(1000);  // Pause
    leftEncoder.reset();
    rightEncoder.reset();

    // --- Step 2: Perform 180-degree turn ---
    Serial.println("Performing 180-degree turn...");
    turnController.turnDegrees(180, 70);

    delay(1000);  // Pause
    leftEncoder.reset();
    rightEncoder.reset();

    // --- Step 3: Drive back ---
    Serial.println("Driving back...");
    driveStraight(forwardTicks, 70);

    Serial.println("Test complete!");
}