////////////////////////////////////////////////////////////////
// Libraries
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include "src/Ball.h"
#include "src/ReferenceTrajectory.h"
#include "src/CommandIssuer.h"
#include "src/ReadingConverter.h"
#include "src/Controller.h"
#include "src/Estimator.h"

////////////////////////////////////////////////////////////////
// External constants
extern const int SERVO_CMD_REST;
extern const byte SERVO_PIN;
extern const unsigned long BAUD_RATE;
extern const unsigned int DT;
extern const BLA::Matrix<2, 2> A;
extern const BLA::Matrix<2, 1> B;
extern const BLA::Matrix<1, 2> C;
extern const BLA::Matrix<1, 4> K;
extern const BLA::Matrix<2, 1> L;

////////////////////////////////////////////////////////////////
// Variables
float setpoint;
int reading;
float observation;
BLA::Matrix<4> state_estimate;
float error;
float action;
unsigned int servo_cmd;
bool saturated = false;
bool ball_removed;
unsigned long now = millis();
unsigned long last_time = millis();
unsigned long time_change = 0;

////////////////////////////////////////////////////////////////
// Initialize objects
VL53L0X sensor;
Servo servo;
Ball ball;
ReferenceTrajectory reference_trajectory;
CommandIssuer command_issuer(servo);
ReadingConverter reading_converter;

//PIDController controller(0.8, 0.2, 0.3);
//PIDEstimator estimator(0.5, 0.2, 0.001*DT);
LinearController controller(K);
LinearEstimator estimator(A, B, C, L, 0.001*DT);


void setup() {
  // Begin I2C communication
  Wire.begin();
  delay(10);

//  // Begin serial connection
//  Serial.begin(BAUD_RATE);
//  delay(10);

  // Start the sensor
  sensor.setTimeout(250);
  while (!sensor.init()) {
//    Serial.println("Failed to detect and initialize sensor VL53L0X!");
    delay(1000);
    }
  sensor.setMeasurementTimingBudget(20000); // Set sensor timing budget, in microseconds (20000 is the minimum)
  sensor.startContinuous();
  delay(10);

  // Start the servo
  servo.attach(SERVO_PIN);  // Attach servo object
  servo.writeMicroseconds(SERVO_CMD_REST);
  delay(10);

//  // Diagnostic printing
//  printer.print_header();
}


void loop() {
  now = millis();
  time_change = now - last_time;
  if (time_change >= DT) {
    setpoint = reference_trajectory.update_setpoint();
    reading = sensor.readRangeContinuousMillimeters();
    observation = reading_converter.update_observation(reading);
    ball_removed = ball.update_removed_status(observation);
    error = estimator.update_error(observation, setpoint);
    state_estimate = estimator.update_estimate(action, ball_removed, saturated);
    action = controller.update_control(state_estimate, ball_removed);
    command_issuer.issue_control(action);
    servo_cmd = command_issuer.get_command();
    saturated = command_issuer.get_saturated();
//    printer.print(setpoint, error, state_estimate, action, servo_cmd, time_change);
    last_time = now;
  }
  else {
    delayMicroseconds(1);  // wait until DT has elapsed to ensure a constant refresh rate
  }
}
