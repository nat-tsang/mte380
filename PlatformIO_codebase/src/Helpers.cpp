#include "Helpers.h"

Helpers::Helpers() {}

void Helpers::buttonCheck()
{
    buttonState = digitalRead(START_SIG);
    if (buttonState) {
        while (digitalRead(START_SIG)) {
            Serial.println("Button still pressed. Take finger off.");
        }
        go = !go;
    }
}