#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;  // Create sensor object
Servo servo;  // Create servo object

const int SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;
const byte INT_SIZE = sizeof(int);

// for ANNIMOS DS3218MG
const int SERVO_CMD_MID = 1500;  // in microseconds
const int SERVO_CMD_MIN = SERVO_CMD_MID - 500;  // in microseconds
const int SERVO_CMD_MAX = SERVO_CMD_MID + 500;  // in microseconds
const int SERVO_CMD_REST = SERVO_CMD_MID + 30;  // in microseconds  // make the ball roll down to the end near sensor in the rest state

const int DT = 20;               // in milliseconds

unsigned long now = millis();
unsigned long last = now;
int time_since_last = 0;


void setup() {
  delay(50);
  Wire.begin();             // Begin I2C communication
  delay(50);
  Serial.begin(BAUD_RATE);  // Begin serial connection
  delay(50);
  
  sensor.setTimeout(250);
  while (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor VL53L0X!");
    delay(1000);
    }

  // Set sensor timing budget, in microseconds (20000 is the minimum)
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  servo.attach(SERVO_PIN);  // Attach servo object
  servo.writeMicroseconds(SERVO_CMD_REST);
  delay(50);
}


void loop() {
  now = millis();
  time_since_last = now - last;
  if (time_since_last >= DT) {
    // Take a sensor reading
    int reading = sensor.readRangeContinuousMillimeters();
    
    // Write sensor reading to serial
    Serial.println(reading);

    // Read servo command from serial
    int servo_cmd = read_int();
    
    // Fail safe to ensure servo pwm is always in the safe range
    servo_cmd = constrain(servo_cmd, SERVO_CMD_MIN, SERVO_CMD_MAX);
    
    // Issue command to servo
    servo.writeMicroseconds(servo_cmd);

    // Store last time
    last = now;
  }
  else
    delayMicroseconds(20);
}


int read_int() {
  int val;
  unsigned char serial_buffer[INT_SIZE];
  if (Serial.readBytes(serial_buffer, INT_SIZE) == INT_SIZE)
    memcpy(&val, serial_buffer, INT_SIZE);
  else
    val = SERVO_CMD_REST;
  return val;
}
