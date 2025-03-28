#pragma once
#include <Pixy2.h>
#include <Arduino.h>
#include <Helpers.h>
#include <tuple>

class PixyLineTracker {     // TODO: Can change class to be PixyColourTracker and pass in the signature
private:
    Pixy2 pixy;
    int lastX;          // Last known x-position of the red blob (centered is 160)
    bool lineDetected;  // Flag to indicate if the red line is detected
    bool bullseyeDetected;
    bool greenBoxDetected;
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

    // Methods
    float readLinePosition(const Block* block, int numBlock);
    std::tuple<int16_t, int16_t> getPixyCoord(int blockSig, const Block* block, int numBlock);
    bool findBullseye(int xCrit, int yCrit, int xLim, int yLim, const Block* block, int numBlock, Motor leftMotor, Motor rightMotor);
    bool findGreenBox(int xCrit, int yCrit, int xLim, int yLim, const Block* block, int numBlock);

    // Getters
    bool getLineDetected() const;
    bool getBullseye() const;
    bool getGreenBox() const;

    // Setters
    void setLineDetected(bool status);
    void setBullseye(bool status);
    void setGreenBox(bool status);

    void setLampON();
    void setLampOFF();
};