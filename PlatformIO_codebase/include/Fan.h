#pragma once
#include <Arduino.h>

class Fan {
private:
    int controlPin;
    bool state;  // true = ON, false = OFF

public:
    // Constructor to set the control pin
    Fan(int pin);

    // Turn the fan ON
    void turnOn();

    // Turn the fan OFF
    void turnOff();

    // Get the current fan state
    bool isOn() const;
};