#include "encoderFilter.h"

encoderFilter::encoderFilter() {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        values[i] = 0.0;
    }
}

float encoderFilter::compute(float newValue) {
    values[index] = newValue;
    index = (index + 1) % WINDOW_SIZE;

    float sum = 0.0;
    int count = filled ? WINDOW_SIZE : index;  // Only sum filled elements at startup
    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    if (index == 0) filled = true;
    return sum / count;
}