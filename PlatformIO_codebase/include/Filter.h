#pragma once
#include <Arduino.h>

#include <vector>

class Filter {
private:
    int xBuffer[5];
    bool xIndex = 0;
    int xTotal = 0;  // Stores the last set speed

    float sF = 0.2; // Smoothing factor (0-1)
    float filteredX = 0;
public:
    Filter();

    // Compute the simple moving average
    int computeSMA(int newValue);

    // Compute exponential moving average
    
};