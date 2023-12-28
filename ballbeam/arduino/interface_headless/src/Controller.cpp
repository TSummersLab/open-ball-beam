#include <BasicLinearAlgebra.h>
#include "Controller.h"


////////////////////////////////////////////////////////////////
// LinearController
LinearController::LinearController(BLA::Matrix<1, 4> K) {
  this->K = K;
  this->u = 0.0;
}

float LinearController::update_control(BLA::Matrix<4> z, bool ball_removed) {
  if (ball_removed)
  {
    u = 0.0;
  }
  else
  {
    // state z is {position, velocity, position_sum, current_input} where all distances are measured in meters and time is in seconds
    // Compute control action using LQR gain K on most recent state estimate
    float du = (K * z)(0);
    u += du;
  }
  return u;
}

float LinearController::get_control() {
  return u;
}


////////////////////////////////////////////////////////////////
// PIDController
PIDController::PIDController(float k_p, float k_i, float k_d){
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->u = 0.0;
}

float PIDController::update_control(BLA::Matrix<4> z, bool ball_removed) {
  if (ball_removed) {
    u = 0.0;
  }
  else {
    float up = k_p*z(0);
    float ui = k_i*z(2);
    float ud = k_d*z(1);
    u = up + ui + ud;
  }
  return u;
}

float PIDController::get_control() {
  return u;
}
