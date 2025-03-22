#include "EncoderReader.h"

      
EncoderReader::EncoderReader(int pin1, int pin2)
    : encoder(pin1, pin2), lastPosition(0), lastTime(millis()), lastSpeed(0.0f) {}

void EncoderReader::reset() {
    encoder.write(0);
    lastPosition = 0;
    lastTime = millis();
    lastSpeed = 0.0f;
}

float EncoderReader::computeSpeed() {
    long currentPosition = encoder.read();
    unsigned long currentTime = millis();

    long deltaTicks = currentPosition - lastPosition;
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert ms to sec

    if (deltaTime > 0) {
        // Speed = (delta revolutions) * circumference / delta time
        float revolutions = (float)deltaTicks / 478.0;  // 478 counts per wheel revolution
        float distance = revolutions * 0.0628;          // 0.0628 m per wheel rev
        lastSpeed = distance / deltaTime;               // Speed in m/s
    }

    lastPosition = currentPosition;
    lastTime = currentTime;

    return lastSpeed;
}

float EncoderReader::getSpeed() const {
    return lastSpeed;
}

long EncoderReader::getTicks() {
    return encoder.read();
}