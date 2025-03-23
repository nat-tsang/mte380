#pragma once
#include <Arduino.h>
#include <Config.h>

class Helpers {
    private:
        int buttonState;
        bool go = false; 

    public:
        Helpers();

        void buttonCheck();
};