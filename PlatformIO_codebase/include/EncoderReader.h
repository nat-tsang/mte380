#pragma once
#include <Arduino.h>
#include <Encoder.h>
#include "Config.h"
#include "Filter.h"

class EncoderReader {
private:
    Encoder encoder;
    long lastPosition;
    unsigned long lastTime;
    float lastSpeed;     // Store last computed speed (m/s)
    
    // templated Filter instance (adjust window size as needed)
    Filter<float, 3> speedFilter;  // window size of 3
    
public:
    EncoderReader(int pin1, int pin2);
    void reset();
    float computeSpeed();    // returns speed in m/s
    float getSpeed() const;  // latest speed reading
    long getTicks();   // raw encoder ticks
};