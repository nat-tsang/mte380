#pragma once
#include <Pixy2.h>
#include <Arduino.h>
#include <tuple>

class PixyLineTracker {
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

    // Update and get the x-position of the red line (-160 to 160)
    int readLinePosition();
    std::tuple<uint16_t, uint16_t> getPixyCoord(int blockSig);

    // Check if the red line is detected
    bool isLineDetected() const;
    bool isBullseye() const;
    bool findBullseye(int xCrit, int yCrit, int xLim, int yLim);
};