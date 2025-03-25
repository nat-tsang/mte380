#pragma once
#include <Pixy2.h>
#include <Arduino.h>
#include <tuple>

class PixyLineTracker {     // TODO: Can change class to be PixyColourTracker and pass in the signature
private:
    Pixy2 pixy;
    int lastX;          // Last known x-position of the red blob (centered is 160)
    bool lineDetected;  // Flag to indicate if the red line is detected
    bool bullseyeDetected;
    int blockSig;
    int xCrit;
    int yCrit;
    int xLim;
    int yLim;

public:
    // Constructor takes the color signature number (1-7) assigned in Pixy
    PixyLineTracker();

    // Initialize Pixy
    void begin();

    // Update and get the x-position of the red line (-157.5 to 157.5)
    float readLinePosition();
    std::tuple<int16_t, int16_t> getPixyCoord(int blockSig);

    // Check if the red line is detected
    bool isLineDetected() const;
    bool isBullseye() const;
    bool findBullseye(int xCrit, int yCrit, int xLim, int yLim);

    void setLampON();
    void setLampOFF();
};