#pragma once
#include <Arduino.h>
#include <Servo.h>

class ServoGripper {
private:
    Servo gripperServo;
    int servoPin;
    int openAngle;   // Angle for open position
    int closeAngle;  // Angle for closed position
    bool isClosed;

public:
    // Constructor to set the servo pin and angles
    ServoGripper(int pin, int openPos = 700, int closePos = 2000);

    // Initialize the servo
    void attach();

    // Open the gripper
    void open();

    // Close the gripper
    void close();

    // Check if gripper is closed
    bool getState() const;
};