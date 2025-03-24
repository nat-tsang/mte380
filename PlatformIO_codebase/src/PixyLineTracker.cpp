#include "PixyLineTracker.h"

PixyLineTracker::PixyLineTracker(uint8_t sig) : signature(sig), lastX(160) {}

void PixyLineTracker::begin() {
    pixy.init();
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

bool PixyLineTracker::isLineDetected() const {
    return lineDetected;
}