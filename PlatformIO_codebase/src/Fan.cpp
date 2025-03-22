#include "Fan.h"

Fan::Fan(int pin) : controlPin(pin), state(false) {
    pinMode(controlPin, OUTPUT);
    digitalWrite(controlPin, LOW);  // Fan starts OFF
}

void Fan::turnOn() {
    digitalWrite(controlPin, HIGH);
    state = true;
}

void Fan::turnOff() {
    digitalWrite(controlPin, LOW);
    state = false;
}

bool Fan::isOn() const {
    return state;
}