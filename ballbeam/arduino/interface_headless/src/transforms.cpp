#include "transforms.h"
#include "constants.h"

extern const int SERVO_CMD_MID;
extern const float READING_OFFSET;
extern const float READING_SCALE;
extern const float OBSERVATION_SCALE;
extern const float OBSERVATION_SCALE_INV_THOU;
extern const float BEAM_ANGLE_SCALE;
extern const float PWM_SCALE;
extern const float PWM_SCALE_INV;
extern const float BEAM_ANGLE_MIN;
extern const float BEAM_ANGLE_MAX;
extern const float A2A_COEFFS[];
extern const float R2O_COEFFS[];


// Convex combination of two numbers a and b, by mixing ratio x
float mix(float a, float b, float x) {
	return x * a + (1 - x) * b;
}


// Soft thresholding operation on number a, with threshold x
// e.g. https://www.mathworks.com/help/wavelet/ref/wthresh.html
float soft(float a, float x) {
	if (a < -x) { return a + x; }
	else if (a > x) { return a - x; }
	else { return 0; }
}


// Degree-5 polynomial evaluation
// c[] should be a float array with 5+1 elements representing the coefficients of the powers of x in descending degree
float polyval5(float c[], float x) {
  float t1 = x;
  float t2 = t1 * t1;
  float t3 = t1 * t2;
  float t4 = t1 * t3;
  float t5 = t1 * t4;
  return c[0] * t5 + c[1] * t4 + c[2] * t3 + c[3] * t2 + c[4] * t1 + c[5];
}


// Invert nonlinearity of sensor using polynomial from calibration data
float reading2observation(int reading) {
  float x = (reading - READING_OFFSET)*READING_SCALE;
  float y = polyval5(R2O_COEFFS, x);
  return OBSERVATION_SCALE_INV_THOU*y;
}


// Invert nonlinearity of servo using polynomial from calibration data
long action2actuation(float u) {
  float uc = constrain(u, -1.0, 1.0);
  float beam_angle = asin(uc);
  beam_angle = constrain(beam_angle, BEAM_ANGLE_MIN, BEAM_ANGLE_MAX);
  float x = beam_angle*BEAM_ANGLE_SCALE;
  float y = polyval5(A2A_COEFFS, x);
  long servo_cmd_dev = long(y*PWM_SCALE_INV);
  long servo_cmd = SERVO_CMD_MID + servo_cmd_dev;
  return servo_cmd;
}
