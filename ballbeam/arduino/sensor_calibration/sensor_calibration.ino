#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;  // Create sensor object
Servo servo;  // Create servo object

const int SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;
const byte INT_SIZE = sizeof(int);

const int SERVO_CMD_REST = 1400;  // in microseconds  // make the ball roll down to the end near sensor in the rest state

const int DT = 20;               // in milliseconds

const int dists[] = {14, 30, 40, 50, 60, 70, 80, 90, 100, 105, 110, 120, 130, 140, 150, 160, 170, 196};
//const int dists[] = {105};
const int num_dists = sizeof(dists) / sizeof(int);

unsigned long now = millis();
unsigned long last = now;
int time_since_last = 0;

const int num_measurements = 20;


void setup() {
  delay(50);
  Wire.begin();             // Begin I2C communication
  delay(50);
  Serial.begin(BAUD_RATE);  // Begin serial connection
  delay(50);
  
  Serial.println("Sensor calibration");
  
  sensor.setTimeout(250);
  while (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor VL53L0X!");
    delay(1000);
    }

  // Set sensor timing budget, in microseconds (20000 is the minimum)
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous();

  servo.attach(SERVO_PIN);  // Attach servo object
  delay(500);

  servo.writeMicroseconds(SERVO_CMD_REST);
  
  Serial.print("\n");
  Serial.print("BEGIN DATA COLLECTION\n");
  delay(1000);

  for (int i = 0; i < num_dists; i += 1) {
//    int dist_cur = map(i, 0, num_steps, dist_min, dist_max);
    int dist_cur = dists[i];
    Serial.print("True distance ");
    Serial.print(dist_cur);
    Serial.print("\n");
    
    delay(10000); // This wait time is for you to move the ball to the new position. Modify as needed.   
    
    for (int count = 1; count <= num_measurements; count += 1) {
      // Get a new sensor reading
      int reading = sensor.readRangeContinuousMillimeters();
      Serial.println(reading);
      delay(100);
    }  
    delay(500);  
    Serial.print("\n");
  }
  Serial.print("END DATA COLLECTION\n");
}


void loop() {
  delay(1);
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
