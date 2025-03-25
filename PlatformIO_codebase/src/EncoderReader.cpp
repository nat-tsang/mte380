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
        float revolutions = (float)deltaTicks / COUNTS_PER_WHEEL_REV;
        float distance = revolutions * WHEEL_CIRCUMFERENCE; // meters

        lastSpeed = speedFilter.computeSMA(distance / deltaTime);  // APPLY FILTER HERE, Speed in m/s
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