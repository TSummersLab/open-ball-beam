#include <BasicLinearAlgebra.h>
#include "constants.h"


extern const byte SERVO_PIN = 10;
extern const unsigned long BAUD_RATE = 115200;
extern const unsigned int DT = 20;  // limited by refresh rate of VL53L0X which is 20ms in HIGH_SPEED mode using Pololu library
extern const long TIMEOUT_COUNT = 5;  // number of consecutive timesteps that the sensor must detect in/out of range to switch active stabilization on/off
extern const float OBS_BACKSTOP = 0.090;  // in meters, ensure there is sufficient buffer to prevent spurious resets


//extern const float SETPOINT1 = 0.0;  // in meters
//extern const float SETPOINT2 = 0.0;  // in meters
extern const float SETPOINT1 = -0.050;  // in meters
extern const float SETPOINT2 = 0.050;  // in meters
extern const long SETPOINT_WAIT_TIME = 10000;  // in milliseconds

extern const int SERVO_CMD_MID = 1500;  // in microseconds
extern const int SERVO_CMD_MIN = SERVO_CMD_MID - 500;  // in microseconds
extern const int SERVO_CMD_MAX = SERVO_CMD_MID + 500;  // in microseconds
extern const int SERVO_CMD_REST = SERVO_CMD_MID + 30;  // in microseconds  // make the ball roll down to the end near sensor in the rest state

extern const float READING_OFFSET = 142.454545;  // get this value from sensor_calibration.py as the raw sensor reading when ball physically at DISTANCE_MID
extern const float READING_SCALE = 0.01;
extern const float OBSERVATION_SCALE = 0.01;
extern const float OBSERVATION_SCALE_INV_THOU = 0.001/OBSERVATION_SCALE;

extern const float BEAM_ANGLE_SCALE = 10.0;
extern const float PWM_SCALE = 0.001;
extern const float PWM_SCALE_INV = 1.0/PWM_SCALE;

extern const float BEAM_ANGLE_MIN = -4.0*DEG_TO_RAD;  // in radians
extern const float BEAM_ANGLE_MAX = 4.0*DEG_TO_RAD;  // in radians

extern const float A2A_COEFFS[] = {0.287338, 0.637543, 0.354444, -0.031702, 0.312741, 0.0};  // Coefficients for action2actuation
extern const float R2O_COEFFS[] = {-0.015191, 0.118442, 0.084168, -0.290603, 0.554838, 0.0};  // Coefficients for reading2observation


// Linear state-space model matrices
extern const BLA::Matrix<2, 2> A = { 1.0, 0.01996341,
                                     0.0, 0.9963437 };
extern const BLA::Matrix<2, 1> B = { 0.0, -0.140095 };
extern const BLA::Matrix<1, 2> C = { 1.0, 0.0 };

// Gains
extern const BLA::Matrix<1, 4> K = { 3.792766, 0.797658, 1.696163, -0.424606 };  // Control gain
extern const BLA::Matrix<2, 1> L = { 0.286209, 0.220031 };  // Estimator gain


extern const bool DIAGNOSTIC_PRINT_MASK[8] = { true,
                                               true,
                                               true,
                                               true,
                                               true,
                                               true,
                                               true,
                                               true };
//extern const bool DIAGNOSTIC_PRINT_MASK[8] = {true,
//                                              true,
//                                              true,
//                                              false,
//                                              false,
//                                              false,
//                                              false,
//                                              false};

extern const char HEADER_STRINGS[8][40] = { "Position_(reference)",
                                        "Position_(measured)",
                                        "Position_(estimated)",
                                        "Velocity_(estimated)",
                                        "Integral_(estimated)",
                                        "Control",
                                        "Servo",
                                        "Delta_time_(ms)" };
extern const char SPACER = ' ';
