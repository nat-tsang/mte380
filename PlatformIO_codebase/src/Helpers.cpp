#include "Helpers.h"

Helpers::Helpers() {
    buttonState;
    go = false;
}

bool Helpers::buttonCheck()
{
    buttonState = digitalRead(START_SIG);
    if (buttonState) {
        while (digitalRead(START_SIG)) {
            Serial.println("Button still pressed. Take finger off.");
        }
        go = !go;
    }
    return go;
}