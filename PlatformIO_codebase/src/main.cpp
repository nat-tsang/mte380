#include <Arduino.h>
#include <Encoder.h>
#include <Config.h>

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

Encoder drive1Encoder(ENCODER_IN5, ENCODER_IN6);    // This will also set the pins as INPUT
Encoder drive2Encoder(ENCODER_IN3, ENCODER_IN4);  // left


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(9600);
  Serial.println(result);
  drive1Encoder.write(0);
  drive2Encoder.write(0);
}

void loop() {
  Serial.print(">");
  // Plotter-specific formatted line (starts with '>', uses var:value pairs)
  Serial.print(">");
  Serial.print("encoder1:");
  Serial.print(drive1Encoder.read());
  Serial.print(",");
  Serial.print("encoder2:");
  Serial.print(drive2Encoder.read());
  Serial.println();  // Auto appends \r\n
  delay(50);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

