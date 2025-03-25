#pragma once
#include <Arduino.h>

template <typename T, int WINDOW_SIZE>
class Filter {
private:
    T buffer[WINDOW_SIZE] = {0};
    int index = 0;
    bool filled = false;
    T runningSum = 0;
    float smoothingFactor;
    float emaFiltered = 0;

public:
    Filter(float sF = 0.5) : smoothingFactor(sF) {}

    // Simple Moving Average (SMA) with running sum
    T computeSMA(T newValue) {
        runningSum -= buffer[index];
        buffer[index] = newValue;
        runningSum += newValue;

        index = (index + 1) % WINDOW_SIZE;
        if (index == 0) filled = true;

        int count = filled ? WINDOW_SIZE : index;
        return runningSum / count;
    }

    // Exponential Moving Average (EMA)
    float computeEMA(T newValue) {
        emaFiltered = smoothingFactor * newValue + (1 - smoothingFactor) * emaFiltered;
        return emaFiltered;
    }

    void reset() {
        for (size_t i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0;
        filled = false;
        emaFiltered = 0;
        runningSum = 0;
        index = 0;
    }
};


// #pragma once
// #include <Arduino.h>

// class Filter {
// private:
//     int xBuffer[5];
//     bool xIndex = 0;
//     int xTotal = 0;  // Stores the last set speed
//     float sF = 0.2; // Smoothing factor (0-1)
//     float filteredX = 0;
// public:
//     Filter(float smoothingFactor);

//     // Compute the simple moving average
//     int computeSMA(int newValue);

//     // Compute exponential moving average
//     int computeEMA(int newX); 
// };
