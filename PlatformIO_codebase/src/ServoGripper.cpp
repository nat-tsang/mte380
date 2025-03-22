#include "ServoGripper.h"

ServoGripper::ServoGripper(int pin, int openPos, int closePos)
    : servoPin(pin), openAngle(openPos), closeAngle(closePos), isClosed(false) {}

void ServoGripper::attach() {
    gripperServo.attach(servoPin);
    open();  // Start open
}

void ServoGripper::open() {
    gripperServo.write(openAngle);
    isClosed = false;
}

void ServoGripper::close() {
    gripperServo.write(closeAngle);
    isClosed = true;
}

bool ServoGripper::getState() const {
    return isClosed;
}