#include "PixyLineTracker.h"
#include "Config.h"
#include <tuple>

PixyLineTracker::PixyLineTracker(){}

void PixyLineTracker::begin() {
  pixy.init();
  bullseyeDetected = false;
  greenBoxDetected = false;
}

float PixyLineTracker::readLinePosition(const Block* block, int numBlock) {
    lineDetected = false;//CHECK THIS

    if (numBlock) {
        for (int i = 0; i < numBlock; i++) {
            if (block[i].m_signature == REDLINE_SIG) {
                lastX = block[i].m_x;  // X is 0 (left) to 315 (right)
                lineDetected = true; // Found the red line
            }
        }
    }
    // Map Pixy X (0-315) to -157.5 (left) to +157.5 (right)
    return lastX - X_CENTER;
}

std::tuple<int16_t, int16_t> PixyLineTracker::getPixyCoord(int blockSig, const Block* block, int numBlock) {
    if (numBlock) {
        for (int i = 0; i < numBlock; i++) {
            if (block[i].m_signature == blockSig) {
                int16_t x = block[i].m_x;
                int16_t y = block[i].m_y;   
                return std::make_tuple(x, y);
            }
        }
    }
    return std::make_tuple(-1, -1);
}

bool PixyLineTracker::getLineDetected() const {
    return lineDetected;
}


bool PixyLineTracker::getBullseye() const {
    return bullseyeDetected;
}


bool PixyLineTracker::getGreenBox() const {
    return greenBoxDetected;
}


void PixyLineTracker::setLineDetected(bool status)
{
    lineDetected = status;
}


void PixyLineTracker::setBullseye(bool status)
{
    bullseyeDetected = status;
}


void PixyLineTracker::setGreenBox(bool status)
{
    greenBoxDetected = status;
}



void PixyLineTracker::setGreenBox(bool status)
{
    greenBoxDetected = status;
}


bool PixyLineTracker::findBullseye(int xCrit, int yCrit, int xLim, int yLim, const Block* block, int numBlock, Motor leftMotor, Motor rightMotor) {
    if (numBlock) {
        for (int i = 0; i < numBlock; i++) {
            if (block[i].m_signature == BULLSEYE_SIG) {
                // bullseyeDetected = true;
                leftbasePWM = 50;
                rightbasePWM = 50;
                rightMotor.setSpeed(50);
                int x_range = abs(xCrit - block[i].m_x);
                if (block[i].m_y > yCrit && x_range < xLim) {
                    bullseyeDetected = true;
                }
            }
        }
    }
    return bullseyeDetected;
}


bool PixyLineTracker::findGreenBox(int xCrit, int yCrit, int xLim, int yLim, const Block* block, int numBlock) {
    if (numBlock) {
        for (int i = 0; i < numBlock; i++) {
            if (block[i].m_signature == GREEN_BOX_SIG) {
                int centroidX = block[i].m_x;
                int centroidY = block[i].m_y;
                if (centroidY > 70) { // Check if the centroid is within the specified y limits
                    greenBoxDetected = true;
                }
            }
        }
    }
    return greenBoxDetected;
}

void PixyLineTracker::setLampON()
{
    pixy.setLamp(1, 0);
}

void PixyLineTracker::setLampOFF()
{
    pixy.setLamp(0, 0);
}
