#include <BasicLinearAlgebra.h>
#include "constants.h"

using namespace BLA;  // Functions in BasicLinearAlgebra are wrapped up inside the namespace BLA


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
