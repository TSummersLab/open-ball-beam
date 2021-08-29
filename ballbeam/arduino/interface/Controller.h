#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;  // Functions in BasicLinearAlgebra are wrapped up inside the namespace BLA

//class Controller {
//  private:
//    
//  public:
//    
//
//};

class LinearController {
  private:
    BLA::Matrix<1, 4> K;
    float u;
  
  public:
    LinearController(BLA::Matrix<1, 4> K);
    void update(BLA::Matrix<4> z, bool ball_removed);
    float get_control();
};


class PIDController {
  private:
    float k_p;
    float k_d;
    float k_i;
    float u;
  
  public:
    PIDController(float k_p, float k_i, float k_d);
    void update(BLA::Matrix<4> z, bool ball_removed);
    float get_control();
};

#endif
