#pragma once
#include <Arduino.h>
#include <Config.h>

class Helpers {
    private:
        int buttonState;

    public:
        Helpers();
        bool go; 
        bool buttonCheck();
};