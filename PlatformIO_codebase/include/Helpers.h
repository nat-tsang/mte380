#pragma once
#include <Arduino.h>

class Helpers {
    private:
        int buttonState;

    public:
        Helpers();
        bool go; 
        bool buttonCheck();
};