#include "ReferenceTrajectory.h"


extern const float SETPOINT1;  // in meters
extern const float SETPOINT2;  // in meters
extern const long SETPOINT_WAIT_TIME;  // in milliseconds

ReferenceTrajectory::ReferenceTrajectory() {
    this->setpoint = SETPOINT1;
    this->setpoint1 = SETPOINT1;
    this->setpoint2 = SETPOINT2;
    this->last = millis();
    this->now = this->last;
    this->change = 0;
    this->wait = SETPOINT_WAIT_TIME;
}

ReferenceTrajectory::ReferenceTrajectory(unsigned long wait = SETPOINT_WAIT_TIME, float setpoint1 = SETPOINT1, float setpoint2 = SETPOINT2) {
  this->setpoint = setpoint1;
  this->setpoint1 = setpoint1;
  this->setpoint2 = setpoint2;
  this->last = millis();
  this->now = this->last;
  this->change = 0;
  this->wait = wait;
}

float ReferenceTrajectory::update_setpoint() {
  now = millis();
  change = now - last;

  if (change > wait)
  {
    if (setpoint == setpoint1)
    {
      setpoint = setpoint2;
    }
    else
    {
      setpoint = setpoint1;
    }
    last = now;
  }
  return setpoint;
}

float ReferenceTrajectory::get_setpoint() {
  return setpoint;
}
