#ifndef MY_REFTRAJ_H
#define MY_REFTRAJ_H

#include <Arduino.h>


// Reference trajectory class
class ReferenceTrajectory {
  private:
    float t;
    float setpoint;
    float setpoint1;
    float setpoint2;
    unsigned long last;
    unsigned long now;    
    unsigned long change;
    unsigned long wait;    
  
  public:
    ReferenceTrajectory(unsigned long wait, float setpoint1, float setpoint2);
  
  void update();

  float get_setpoint();
};

#endif
