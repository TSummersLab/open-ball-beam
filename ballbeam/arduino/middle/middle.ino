#include <Servo.h>

Servo servo;

const int SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;
//const int SERVO_CMD = 1500;  // in microseconds
const int SERVO_CMD = 1400;  // in microseconds

void setup() {
  Serial.begin(BAUD_RATE);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(SERVO_CMD);
}

void loop() {
  servo.writeMicroseconds(SERVO_CMD);
}
