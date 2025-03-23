#pragma once
#include <Arduino.h>

class Helpers {
    private:
        int buttonState;
        bool go = false; 

    public:
        Helpers();

        void buttonCheck();
};