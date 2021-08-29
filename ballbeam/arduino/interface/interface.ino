////////////////////////////////////////////////////////////////
// Libraries
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
#include "Ball.h"
#include "ReferenceTrajectory.h"
#include "Issuer.h"
#include "ReadingConverter.h"
#include "Controller.h"
#include "Estimator.h"

using namespace BLA;  // Functions in BasicLinearAlgebra are wrapped up inside the namespace BLA


////////////////////////////////////////////////////////////////
// Constants
const byte SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;

const unsigned int DT = 20;  // limited by refresh rate of VL53L0X which is 20ms in HIGH_SPEED mode using Pololu library

const int TIMEOUT_COUNT = 5;  // number of consecutive timesteps that the sensor must detect in/out of range to switch active stabilization on/off

const float OBS_BACKSTOP = 0.090;  // in m, ensure there is sufficient buffer to prevent spurious resets

//const float SETPOINT1 = 0.0;
//const float SETPOINT2 = 0.0;
const float SETPOINT1 = -0.050;
const float SETPOINT2 = 0.050;

const long SETPOINT_WAIT_TIME = 10000;  // in milliseconds



// Linear state-space model matrices
// NOTE: I had trouble trying to put these in constants.cpp and importing them using the extern keyword - maybe a namespace issue using BLA:: ?
BLA::Matrix<2, 2> A = {1.0, 0.01996341,
                       0.0, 0.9963437
                      };
BLA::Matrix<2, 1> B = { 0.0,
                       -0.140095
                      };
BLA::Matrix<1, 2> C = {1.0, 0.0};

// Gains
BLA::Matrix<1, 4> K = {3.792766, 0.797658, 1.696163, -0.424606};  // Control gain
BLA::Matrix<2, 1> L = {0.286209, 0.220031};  // Estimator gain



//const bool DIAGNOSTIC_PRINT_MASK[] = {true, 
//                                      true, 
//                                      true, 
//                                      true, 
//                                      true, 
//                                      true,
//                                      true, 
//                                      true};
const bool DIAGNOSTIC_PRINT_MASK[] = {true, 
                                      true, 
                                      true, 
                                      false, 
                                      false, 
                                      false,
                                      false, 
                                      false};
const char *HEADER_STRINGS[] = {"Position_(reference)",
                                "Position_(measured)",
                                "Position_(estimated)", 
                                "Velocity_(estimated)", 
                                "Integral_(estimated)", 
                                "Control",
                                "Servo",
                                "Delta_time_(ms)"};
const char SPACER = ' ';


////////////////////////////////////////////////////////////////
// External constants
extern const int SERVO_CMD_MID;
extern const int SERVO_CMD_MIN;
extern const int SERVO_CMD_MAX;
extern const int SERVO_CMD_REST;


////////////////////////////////////////////////////////////////
// Variables
float setpoint = SETPOINT1;
int reading;
float observation;
BLA::Matrix<4> state_estimate = {0.0, 0.0, 0.0, 0.0};
float error = 0.0;
float action = 0.0;
unsigned int servo_cmd = SERVO_CMD_MID;
bool saturated = false;
bool ball_removed = false;

unsigned long now = millis();
unsigned long last_time = millis();
unsigned long time_change = 0;


////////////////////////////////////////////////////////////////
// Function definitions
void diagnostic_print_header() {
  for (int i = 0; i < 8; i++) {
    if (DIAGNOSTIC_PRINT_MASK[i]) {
      Serial.print(HEADER_STRINGS[i]);
      Serial.print(SPACER);
    }
  }
  Serial.print("\n");
}


void diagnostic_print(float vals[]) {    
  for (int i = 0; i < 8; i++) {
    if (DIAGNOSTIC_PRINT_MASK[i]) {
      Serial.print(vals[i], 6);
      Serial.print(SPACER);
    }
  }
  Serial.print("\n");
}


// Instantiate objects
VL53L0X sensor;
Servo servo;
Ball ball(ball_removed, OBS_BACKSTOP, TIMEOUT_COUNT);
ReferenceTrajectory reference_trajectory(SETPOINT_WAIT_TIME, SETPOINT1, SETPOINT2);
Issuer issuer(servo, SERVO_CMD_MID, SERVO_CMD_MIN, SERVO_CMD_MAX);


//PIDController controller(0.8, 0.3, 0.4);
//PIDEstimator estimator(0.50, 0.25, 0.001*DT);
LinearController controller(K);
LinearEstimator estimator(A, B, C, L, 0.001*DT);

//// TODO - make a base class to achieve functionality like below
//bool USE_PID = true;
//if (USE_PID) {
//  PIDController controller(0.8, 0.3, 0.4);
//  PIDEstimator estimator(0.50, 0.25, 0.001*DT);
//}
//else {
//  LinearController controller(K);
//  LinearEstimator estimator(A, B, C, L, 0.001*DT);
//}

ReadingConverter reading_converter;


void setup() {  
  // Begin I2C communication
  Wire.begin();
  delay(10);
  
  // Begin serial connection
  Serial.begin(BAUD_RATE);
  delay(10);
  
  // Start the sensor
  sensor.setTimeout(250);
  while (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor VL53L0X!");
    delay(1000);
    }  
  sensor.setMeasurementTimingBudget(20000); // Set sensor timing budget, in microseconds (20000 is the minimum)
  sensor.startContinuous();
  delay(10);

  // Start the servo
  servo.attach(SERVO_PIN);  // Attach servo object
  servo.writeMicroseconds(SERVO_CMD_REST);
  delay(10);
  
  diagnostic_print_header();
}


void loop() {
  now = millis();
  time_change = now - last_time;
  
  if (time_change >= DT) {    
    // Take a measurement
    reading = sensor.readRangeContinuousMillimeters();
    
    // Convert raw reading to standard observation
    reading_converter.update(reading);
    observation = reading_converter.get_observation();

    // Update ball removed status
    ball.update(observation);
    ball_removed = ball.is_removed();

    // Update reference trajectory
    reference_trajectory.update();
    setpoint = reference_trajectory.get_setpoint();

    // Update state estimator
    estimator.update_error(observation, setpoint);
    estimator.update_estimate(action, ball_removed, saturated);
    error = estimator.get_error();
    state_estimate = estimator.get_estimate();

    // Update controller
    controller.update(state_estimate, ball_removed);
    action = controller.get_control();
    
    // Issue control
    issuer.issue_control(action);
    servo_cmd = issuer.get_cmd();
    saturated = issuer.get_saturated();

    // Diagnostic printing
    float print_vals[] = {1000*setpoint, 1000*(error + setpoint), 1000*(state_estimate(0) + setpoint), 50*state_estimate(1), 50*state_estimate(2), 500*action, 0.*float(servo_cmd), float(time_change)};
    diagnostic_print(print_vals);
    
    last_time = now;
  }
  else {
    delayMicroseconds(1);  // wait until DT has elapsed to ensure a constant refresh rate
  }
}
