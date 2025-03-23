#include "Filter.h"

Filter::Filter(float smoothingFactor){
    xIndex = 0;
    xTotal = 0;
    sF = smoothingFactor;
    filteredX = 0;

    // init vector 
    for (int i = 0; i < 5; i++) {
        xBuffer[i] = 0;
    }
}

int Filter::computeSMA(int newValue)    // Simple Moving Average
{
    xTotal -= xBuffer[xIndex];
    xBuffer[xIndex] = newValue;
    xTotal += newValue;
    xIndex = (xIndex + 1) % 5;
    return xTotal / 5;
}

int Filter::computeEMA(int newX)    // Exponential Moving Average
{
    return sF * newX + (1-sF) * filteredX;
}
