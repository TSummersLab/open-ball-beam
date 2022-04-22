#include <Servo.h>

Servo servo;

const int SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;

const float frequency = 0.1;  // in Hz

unsigned long initial_time = millis();
unsigned long now = initial_time;
unsigned long last = initial_time;
long time_since_last = 0;
long time_since_begin = 0;

const int SERVO_CMD_MIN = 500;  // in microseconds
const int SERVO_CMD_MAX = 2500;  // in microseconds

const int DT = 20;               // in milliseconds

double x;
long y;
long servo_out;

void setup() {
  Serial.begin(BAUD_RATE);
  servo.attach(SERVO_PIN);
}

void loop() {
  now = millis();
  time_since_last = now - last;
  if (time_since_last >= DT){
    Serial.print(time_since_last);
    Serial.print(" ");
    time_since_begin = now - initial_time;  
    x = sin(2*PI*frequency*time_since_begin/1000);
    y = 1000*x;
    servo_out = map(y, -1000, 1000, SERVO_CMD_MIN, SERVO_CMD_MAX);
    Serial.println(servo_out);
    servo.writeMicroseconds(servo_out);    
    last = now;
  }
  else{
    delay(1);
  }
}
