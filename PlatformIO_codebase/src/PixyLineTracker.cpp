#include "PixyLineTracker.h"
#include "Config.h"
#include <tuple>

PixyLineTracker::PixyLineTracker() : lastX(0) {}

void PixyLineTracker::begin() {
    pixy.init();
    bullseyeDetected = true;
}

int PixyLineTracker::readLinePosition() {
    pixy.ccc.getBlocks();
    lineDetected = false;

    if (pixy.ccc.numBlocks) {
        // Find the largest block of the red line (signature match)
        int largestArea = 0;
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == REDLINE_SIG) {
                int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
                if (area > largestArea) {
                    largestArea = area;
                    lastX = pixy.ccc.blocks[i].m_x;  // X is 0 (left) to 319 (right)
                    lineDetected = true; // Found the red line
                }
            }
        }
    }
    // Map Pixy X (0-315) to -157.5 (left) to +157.5 (right)
    return lastX - X_CENTER;
}

std::tuple<int16_t, int16_t> PixyLineTracker::getPixyCoord(int blockSig)
{
    pixy.ccc.getBlocks();
    Serial.println(pixy.ccc.numBlocks);
    Serial.println("hi");
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == blockSig) {
                Serial.println("block found.");
                int16_t x = pixy.ccc.blocks->m_x;
                int16_t y = pixy.ccc.blocks->m_y;
                return std::make_tuple(x, y);
            }
        }
    }
    return std::make_tuple(-1, -1);
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
    Serial.println(pixy.ccc.numBlocks);
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == BULLSEYE_SIG) {
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
