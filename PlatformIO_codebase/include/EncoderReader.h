#pragma once
#include <Arduino.h>
#include <Encoder.h>

class EncoderReader {
private:
    Encoder encoder;
    long lastPosition;
    unsigned long lastTime;
    float lastSpeed;     // Store last computed speed (m/s)

    static constexpr float COUNTS_PER_WHEEL_REV = 478.0; // 12 CPR * 4 * 9.96 gear ratio
    static constexpr float WHEEL_CIRCUMFERENCE = 0.0628; // meters (pi * 0.02m wheel)
    
public:
    EncoderReader(int pin1, int pin2);
    void reset();
    float computeSpeed();    // returns speed in m/s
    float getSpeed() const;  // latest speed reading
    long getTicks();   // raw encoder ticks
};