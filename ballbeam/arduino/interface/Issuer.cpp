#include <Servo.h>
#include "Issuer.h"
#include "transforms.h"


Issuer::Issuer(Servo servo, unsigned int servo_cmd_mid, unsigned int servo_cmd_min, unsigned int servo_cmd_max) {
  this->servo = servo;
  this->servo_cmd_mid = servo_cmd_mid;
  this->servo_cmd_min = servo_cmd_min;
  this->servo_cmd_max = servo_cmd_max;
  this->servo_cmd = servo_cmd_mid;
  this->saturated = false;      
}

void Issuer::issue_control(float u) {
  long a = action2actuation(u);
  saturated = (a <= servo_cmd_min || a >= servo_cmd_max);
  servo_cmd = constrain(a, servo_cmd_min, servo_cmd_max);  
  servo.writeMicroseconds(servo_cmd);  // Send control command to servo    
}

unsigned int Issuer::get_cmd() {
  return servo_cmd;
}

bool Issuer::get_saturated() {
  return saturated;
}
