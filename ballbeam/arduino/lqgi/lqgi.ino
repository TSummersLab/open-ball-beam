#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

// Functions in BasicLinearAlgebra are wrapped up inside the namespace BLA
using namespace BLA;

VL53L0X sensor;  // create sensor object
Servo servo;  // create servo object

const int SERVO_PIN = 10;
const unsigned long BAUD_RATE = 115200;
const float RAD2DEG = 360.0 / (2 * PI);

const int DT = 20;  // limited by refresh rate of VL53L0X which is 20ms in HIGH_SPEED mode using Pololu library
unsigned long now = millis();
unsigned long last_time = millis();
unsigned long time_change = 0;

const int timeout_steps = 5;  // number of consecutive timesteps that the sensor must detect in/out of range to switch active stabilization on/off
int timeout = 0;  // timeout counter
const float obs_midpoint = 0.105;  // in mm
const float obs_backstop = obs_midpoint + 0.090;  // in m, ensure there is sufficient buffer to prevent spurious resets
const float setpoint1 = 0.0;
const float setpoint2 = 0.0;
//const float setpoint1 = -0.040;
//const float setpoint2 = 0.040;
int reading = 0;
float setpoint = setpoint1;
float observation;


const float READING_OFFSET = 142.454545;  // get this value from sensor_calibration.py as the raw sensor reading when ball physically at DISTANCE_MID
const float READING_SCALE = 0.01;
const float OBSERVATION_SCALE = 0.01;
const float OBSERVATION_SCALE_INV_THOU = 0.001/OBSERVATION_SCALE;

const float BEAM_ANGLE_SCALE = 10.0;
const float PWM_SCALE = 0.001;
const float PWM_SCALE_INV = 1.0/PWM_SCALE;

unsigned long setpoint_now = millis();
unsigned long setpoint_last_time = millis();
unsigned long setpoint_wait_time = 10000;  // in milliseconds

bool saturated = false;
bool ball_removed = false;

// for ANNIMOS DS3218MG
const int SERVO_CMD_MID = 1500;  // in microseconds
const int SERVO_CMD_MIN = SERVO_CMD_MID - 500;  // in microseconds
const int SERVO_CMD_MAX = SERVO_CMD_MID + 500;  // in microseconds
const int SERVO_CMD_REST = SERVO_CMD_MID + 30;  // in microseconds  // make the ball roll down to the end near sensor in the rest state
int servo_cmd = SERVO_CMD_MID;

// Coefficients for action2actuation
const float a2a_coeffs[] = {0.287338, 0.637543, 0.354444, -0.031702, 0.312741};

// Coefficients for reading2observation
const float r2o_coeffs[] = {-0.015191, 0.118442, 0.084168, -0.290603, 0.554838};

float error = 0;
float error_sum = 0;

float u = 0;
float du = 0;

BLA::Matrix<2> x = {0.0, 0.0};
BLA::Matrix<4> z = {0.0, 0.0, 0.0, 0.0};
BLA::Matrix<1> y = {0.0};

BLA::Matrix<2, 2> A = {1.0, 0.01996341,
                       0.0, 0.9963437
                      };
BLA::Matrix<2, 1> B = { 0.0,
                        -0.140095
                      };
BLA::Matrix<1, 2> C = {1.0, 0.0};

BLA::Matrix<1, 4> K = {4.767218, 0.970734, 4.600305, -0.470930};  // Control gain
BLA::Matrix<2, 1> L = {0.286209, 0.220031};  // Estimator gain

BLA::Matrix<2, 2> AL = A - L * C;


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
  Serial.print("Position_(measured)");
  Serial.print(" ");
  Serial.print("Position_(estimated)");
  Serial.print(" ");
  Serial.print("Velocity_(estimated)");
  Serial.print(" ");
  Serial.print("Integral_(estimated)");
  Serial.print(" ");
  Serial.print("Control");
  Serial.print("\n");
  
  servo.attach(SERVO_PIN);  // Attach servo object
  servo.writeMicroseconds(SERVO_CMD_REST);
  delay(50);
}


void loop() {
  now = millis();
  time_change = now - last_time;
  if (time_change >= DT) {
    // Control
    if (ball_removed)
    {
      // Use zero control input
      u = 0.0;
      saturated = issue_control(u);

      // Reset state to zero
      z = {0.0, 0.0, 0.0, 0.0};
    }
    else
    {
      // state z is {position, velocity, position_sum, current_input} where all distances are measured in meters and time is in seconds
      // Compute control action using LQR gain K on most recent state estimate
      du = (K * z)(0);
      u += du;
      saturated = issue_control(u);
    }

    // Take a measurement
    reading = sensor.readRangeContinuousMillimeters();
    
    // Convert raw reading to standard observation
    observation = reading2observation(reading);

    // Ball removed logic
    if (!ball_removed) {
      if (observation > obs_backstop) {
        timeout += 1;
      }
      else {
        timeout = 0;
      }
    }
    else {
      if (observation <= obs_backstop) {
        timeout -= 1;
      }
      else {
        timeout = 2 * timeout_steps;
      }
    }
    ball_removed = timeout > timeout_steps;


    // Choose the setpoint
    setpoint_now = millis();
    unsigned long setpoint_time_change = setpoint_now - setpoint_last_time;
    if (setpoint_time_change > setpoint_wait_time)
    {
      if (setpoint == setpoint1)
      {
        setpoint = setpoint2;
      }
      else
      {
        setpoint = setpoint1;
      }
      setpoint_last_time = setpoint_now;
    }

    if (ball_removed)
    {
      error = 0;
      error_sum = 0;
    }
    else
    {      
      error = observation - setpoint;
      if (!saturated) {
        error_sum += error*(0.001*DT);  // Anti-windup logic
      }
    }
    
    error = observation;
    y = {error};   

    // Update state estimate using LQE gain L
    BLA::Matrix<2> x_prev = z.Submatrix<2,1>(0,0);
    x = AL * x_prev + B * u + L * y;
    z(0) = x(0);
    z(1) = x(1);
    z(2) = error_sum;
    z(3) = u;

    Serial.print(1000 * y(0), 6);
    Serial.print(" ");
    Serial.print(1000 * z(0), 6);
    Serial.print(" ");
    Serial.print(50 * z(1), 6);
    Serial.print(" ");
    Serial.print(50 * z(2), 6);
    Serial.print(" ");
    Serial.print(500 * u, 6);
    Serial.print("\n");
    //    Serial.print(reading);
    //    Serial.print(time_change)
    //    Serial.print(ball_removed)
    last_time = now;
  }
  else {
    delayMicroseconds(1);  // wait until DT has elapsed to ensure a constant refresh rate
  }
}

// Degree-5 polynomial evaluation
float polyval5(float coeffs[], float x) {
  float t1 = x;
  float t2 = t1 * t1;
  float t3 = t1 * t2;
  float t4 = t1 * t3;
  float t5 = t1 * t4;
  return coeffs[0] * t5 + coeffs[1] * t4 + coeffs[2] * t3 + coeffs[3] * t2 + coeffs[4] * t1;
}


// Invert nonlinearity of sensor using cubic polynomial from calibration data
float reading2observation(int reading) {
  float x = (reading - READING_OFFSET)*READING_SCALE;
  float y = polyval5(r2o_coeffs, x);
  return OBSERVATION_SCALE_INV_THOU*y;
}


// Invert nonlinearity of servo using cubic polynomial from calibration data
int action2actuation(float u) {
  float uc = constrain(u, -1.0, 1.0);
  float beam_angle = asin(u);
  float x = beam_angle*BEAM_ANGLE_SCALE;
  float y = polyval5(a2a_coeffs, x);
  return int(y*PWM_SCALE_INV);
}


// Send control command to servo
bool issue_control(float u) {
  int servo_cmd_dev = action2actuation(u);
  servo_cmd = SERVO_CMD_MID + servo_cmd_dev;

  if (servo_cmd > SERVO_CMD_MAX) {
    servo_cmd = SERVO_CMD_MAX;
    saturated = true;
  }
  else if (servo_cmd < SERVO_CMD_MIN) {
    servo_cmd = SERVO_CMD_MIN;
    saturated = true;
  }
  else {
    saturated = false;
  }
  servo.writeMicroseconds(servo_cmd);
  return saturated;
}
