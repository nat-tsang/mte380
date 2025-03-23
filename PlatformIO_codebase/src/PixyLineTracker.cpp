#include "PixyLineTracker.h"

PixyLineTracker::PixyLineTracker(uint8_t sig) : signature(sig), lastX(0) {}

void PixyLineTracker::begin() {
    pixy.init();
    sigBullseye = 2;
    bullseyeDetected = false;
}

int PixyLineTracker::readLinePosition() {
    pixy.ccc.getBlocks();
    lineDetected = false;

    if (pixy.ccc.numBlocks) {
        // Find the largest block of the red line (signature match)
        int largestArea = 0;
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == signature) {
                int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
                if (area > largestArea) {
                    largestArea = area;
                    lastX = pixy.ccc.blocks[i].m_x;  // X is 0 (left) to 319 (right)
                    lineDetected = true; // Found the red line
                }
            }
        }
    }
    // Map Pixy X (0-319) to -160 (left) to +160 (right)
    return lastX - 160;
}

int PixyLineTracker::getPixyX(int blockSig)
{
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == blockSig) {
                return pixy.ccc.blocks->m_x;
            }
        }
    }
    return -1; 
}

bool PixyLineTracker::isLineDetected() const {
    return lineDetected;
}

bool PixyLineTracker::isBullseye() const
{
    return bullseyeDetected;
}

bool PixyLineTracker::findBullseye(int xCrit, int yCrit, int xLim, int yLim)
{
    pixy.ccc.getBlocks();
    
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == sigBullseye) {
                int x_range = abs(xCrit - pixy.ccc.blocks[i].m_x);
                int y_range = abs(yCrit - pixy.ccc.blocks[i].m_y);
                if (x_range < xLim && y_range < yLim) {
                    bullseyeDetected = true;
                }
            }
        }
    }
    return bullseyeDetected;
}
