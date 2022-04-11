#ifndef MY_REFTRAJ_H
#define MY_REFTRAJ_H

#include <Arduino.h>


// Reference trajectory class
class ReferenceTrajectory {
  private:
    float setpoint;
    float setpoint1;
    float setpoint2;
    unsigned long last;
    unsigned long now;    
    unsigned long change;
    unsigned long wait;    
  
  public:
    ReferenceTrajectory();
    ReferenceTrajectory(unsigned long wait, float setpoint1, float setpoint2);
    float get_setpoint();
    float update_setpoint();
};

#endif
