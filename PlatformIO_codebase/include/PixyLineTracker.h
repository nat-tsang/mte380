#pragma once
#include <Pixy2.h>
#include <Arduino.h>

class PixyLineTracker {
private:
    Pixy2 pixy;
    uint8_t signature;  // Color signature number for the red line
    int lastX;          // Last known x-position of the red blob (centered is 160)
    bool lineDetected;  // Flag to indicate if the red line is detected
    bool bullseyeDetected;
    int sigBullseye;
    int blockSig
    int xCrit;
    int yCrit;
    int xLim;
    int yLim;

    public:
    // Constructor takes the color signature number (1-7) assigned in Pixy
    PixyLineTracker(uint8_t sig);

    // Initialize Pixy
    void begin();

    // Update and get the x-position of the red line (-160 to 160)
    int readLinePosition();
    int getPixyX(int blockSig);

    // Check if the red line is detected
    bool isLineDetected() const;
    bool isBullseye() const;
    bool findBullseye(int xCrit, int yCrit, int xLim, int yLim);
};