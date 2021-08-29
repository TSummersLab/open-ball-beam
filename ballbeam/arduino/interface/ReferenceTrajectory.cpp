#include "ReferenceTrajectory.h"


ReferenceTrajectory::ReferenceTrajectory(unsigned long wait, float setpoint1, float setpoint2) {
  this->setpoint = setpoint1;
  this->setpoint1 = setpoint1;
  this->setpoint2 = setpoint2;
  this->last = millis();
  this->now = this->last;      
  this->change = 0;      
  this->wait = wait;      
}
  
void ReferenceTrajectory::update() {        
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
}

float ReferenceTrajectory::get_setpoint() {
  return setpoint;
}
