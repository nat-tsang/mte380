#include "PixyLineTracker.h"
#include "Config.h"
#include <tuple>

PixyLineTracker::PixyLineTracker(){}

void PixyLineTracker::begin() {
  pixy.init();
  bullseyeDetected = false;
}

float PixyLineTracker::readLinePosition() {
    pixy.ccc.getBlocks();
    lineDetected = false;//CHECK THIS

    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == REDLINE_SIG) {
                lastX = pixy.ccc.blocks[i].m_x;  // X is 0 (left) to 315 (right)
                lineDetected = true; // Found the red line
            }
        }
        // Find the largest block of the red line (signature match)
        // TODO: Change code because we should only have one red block and do not need for loop
        // int largestArea = 0;
        // for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        //     if (pixy.ccc.blocks[i].m_signature == REDLINE_SIG) {
        //         int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        //         if (area > largestArea) {
        //             largestArea = area;
        //             lastX = pixy.ccc.blocks[i].m_x;  // X is 0 (left) to 319 (right)
        //             lineDetected = true; // Found the red line
        //         }
        //     }
        // }
    }
    // Map Pixy X (0-315) to -157.5 (left) to +157.5 (right)
    return lastX - X_CENTER;
}

std::tuple<int16_t, int16_t> PixyLineTracker::getPixyCoord(int blockSig) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == blockSig) {
                int16_t x = pixy.ccc.blocks->m_x;
                int16_t y = pixy.ccc.blocks->m_y;   //ASK NATALIE ABOUT THIS *CHELS SAYS FIXED BY SIG COUNT
                return std::make_tuple(x, y);
            }
        }
    }
    return std::make_tuple(-1, -1);
}

bool PixyLineTracker::isLineDetected() const {
    return lineDetected;
}

bool PixyLineTracker::isBullseye() const {
    return bullseyeDetected;
}

bool PixyLineTracker::findBullseye(int xCrit, int yCrit, int xLim, int yLim) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == BULLSEYE_SIG) {
                // int x_range = abs(xCrit - pixy.ccc.blocks[i].m_x);
                // int y_range = abs(yCrit - pixy.ccc.blocks[i].m_y);
                if (pixy.ccc.blocks[i].m_y > yCrit) {
                    bullseyeDetected = true;
                }
            }
        }
    }
    return bullseyeDetected;
}
