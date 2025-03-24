#pragma once
#include <Arduino.h>

class SimpleMovingAverage {
private:
    static const int WINDOW_SIZE = 3;  // Tune this based on performance
    float values[WINDOW_SIZE];
    int index = 0;
    bool filled = false;

public:
    SimpleMovingAverage();
    float compute(float newValue);
};