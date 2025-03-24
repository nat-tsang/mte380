#include <Arduino.h>
#include "../include/Config.h"
#include "../src/Motor.cpp"
#include "../src/EncoderReader.cpp"


EncoderReader rightEncoder(ENCODER_IN5, ENCODER_IN6);
EncoderReader leftEncoder(ENCODER_IN3, ENCODER_IN4);

Motor rightMotor(u2_IN1, u2_IN2, true);
Motor leftMotor(u3_IN1, u3_IN2, false);


const int pwmValues[] = {50, 75, 100, 125, 150, 175, 200};  // Sweep range
const int numSteps = sizeof(pwmValues) / sizeof(pwmValues[0]);

void setup() {
    Serial.begin(115200);
    leftEncoder.reset();
}

void loop() {
    for (int i = 0; i < numSteps; i++) {
        int pwm = pwmValues[i];
        // leftMotor.setSpeed(pwm);
        // leftEncoder.reset();
        rightMotor.setSpeed(pwm);
        rightEncoder.reset();
        delay(3000);  // Allow motor to stabilize

        //float speed = leftEncoder.computeSpeed();  // m/s
        float speed = rightEncoder.computeSpeed();  // m/s

        Serial.print("rightPWM: "); Serial.print(pwm);
        Serial.print(", Speed (m/s): "); Serial.print(speed);
        if (speed > 0) {
            float Kf = pwm / speed;
            Serial.print(", Kf estimate: "); Serial.println(Kf);
        } else {
            Serial.println(", Motor did not move");
        }
        delay(1000);  // Pause between tests
    }
    rightMotor.stop();
    leftMotor.stop();
    while (true);  // Stop the loop after testing
}

// right = 50.648 + 50.303 = 50.475
// left = 51.647 + 54.13 = 52.8885