#include <Servo.h>
#include "CommandIssuer.h"
#include "transforms.h"

extern const int SERVO_CMD_MID;
extern const int SERVO_CMD_MIN;
extern const int SERVO_CMD_MAX;

CommandIssuer::CommandIssuer(Servo servo) {
	this->servo = servo;
	this->servo_cmd_mid = SERVO_CMD_MID;
	this->servo_cmd_min = SERVO_CMD_MIN;
	this->servo_cmd_max = SERVO_CMD_MAX;
	this->servo_cmd = SERVO_CMD_MID;
	this->saturated = false;
}

CommandIssuer::CommandIssuer(Servo servo, unsigned int servo_cmd_mid, unsigned int servo_cmd_min, unsigned int servo_cmd_max) {
  this->servo = servo;
  this->servo_cmd_mid = servo_cmd_mid;
  this->servo_cmd_min = servo_cmd_min;
  this->servo_cmd_max = servo_cmd_max;
  this->servo_cmd = servo_cmd_mid;
  this->saturated = false;
}

void CommandIssuer::issue_control(float u) {
  long a = action2actuation(u);
  saturated = (a <= servo_cmd_min || a >= servo_cmd_max);
  servo_cmd = constrain(a, servo_cmd_min, servo_cmd_max);
  servo.writeMicroseconds(servo_cmd);  // Send control command to servo
}

unsigned int CommandIssuer::get_command() {
  return servo_cmd;
}

bool CommandIssuer::get_saturated() {
  return saturated;
}
