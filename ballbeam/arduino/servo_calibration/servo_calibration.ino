#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_LSM6DS33.h>
#include <Servo.h>

VL53L0X sensor;  // create sensor object
Adafruit_LSM6DS33 lsm6ds33;
Servo servo;  // create servo object

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

const int servo_pin = 10;

const int num_steps = 10;
const int num_measurements = 20;

const int servo_out_min = 1000;
const int servo_out_max = 2000;


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Servo angle calibration");

  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS33 Found!");

  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  Serial.println("+-2G");

  lsm6ds33.setAccelDataRate(LSM6DS_RATE_26_HZ);
  Serial.print("Accelerometer data rate set to: ");
  Serial.println("26 Hz");

  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2

  servo.attach(servo_pin);  // attaches the servo to the servo object

  Serial.print("\n");
  Serial.print("BEGIN DATA COLLECTION\n");
  delay(1000);

  for (int i = 0; i <= num_steps; i += 1) {
    int servo_out = map(i, 0, num_steps, servo_out_min, servo_out_max);
    Serial.println(servo_out);
    servo.writeMicroseconds(servo_out);
    delay(1000);

    for (int count = 1; count <= num_measurements; count += 1) {
      // Get a new normalized sensor event
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      lsm6ds33.getEvent(&accel, &gyro, &temp);

      Serial.print(accel.acceleration.x, 6);
      Serial.print(" ");
      Serial.print(accel.acceleration.y, 6);
      Serial.print(" ");
      Serial.print(accel.acceleration.z, 6);
      Serial.print("\n");
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
