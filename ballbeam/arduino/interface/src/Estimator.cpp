#include <BasicLinearAlgebra.h>
#include "Estimator.h"
#include "transforms.h"


////////////////////////////////////////////////////////////////
// LinearEstimator
LinearEstimator::LinearEstimator(BLA::Matrix<2, 2> A, BLA::Matrix<2, 1> B, BLA::Matrix<1, 2> C, BLA::Matrix<2, 1> L, float DT) {
  this->A = A;
  this->B = B;
  this->C = C;
  this->L = L;
  this->AL = A - L * C;
  this->DT = DT;
  this->x = {0.0, 0.0};
  this->z = {0.0, 0.0, 0.0, 0.0};
  this->y = {0.0};
  this->error = 0.0;
}

float LinearEstimator::update_error(float observation, float setpoint) {
  // Compute error as difference between observation and setpoint     
    error = observation - setpoint;
    return error;
}

float LinearEstimator::get_error(){
  return error;
}

BLA::Matrix<4> LinearEstimator::update_estimate(float u, bool ball_removed, bool saturated) {
  if (ball_removed)
  {
    // Reset state estimate to zero
    z = {0.0, 0.0, 0.0, 0.0};
  }
  else
  {       
    // Output vector
    y = {error};   

    // Update physical state estimate using LQE gain L
    BLA::Matrix<2> x_prev = z.Submatrix<2,1>(0,0);
    x = AL * x_prev + B * u + L * y;

    // Construct full state estimate
    // Put physical state in states 0, 1
    z(0) = x(0);
    z(1) = x(1);
    
    // Put error sum in state 2, use anti-windup logic
    if (!saturated) {
      z(2) += error*DT;
    }

    // Put current control input in state 3
    z(3) = u;      
  }   
  return z;
}

BLA::Matrix<4> LinearEstimator::get_estimate() {
  return z;
}



////////////////////////////////////////////////////////////////
// PIDEstimator
PIDEstimator::PIDEstimator(float error_mix, float error_diff_mix, float DT) {
  this->error_mix = error_mix;
  this->error_diff_mix = error_diff_mix;
  this->error = 0.0;
  this->e_last = 0.0;
  this->z = {0.0, 0.0, 0.0, 0.0};
  this->DT = DT;
}

float PIDEstimator::update_error(float observation, float setpoint) {
  // Compute error as difference between observation and setpoint     
    error = observation - setpoint;
    return error;
}

BLA::Matrix<4> PIDEstimator::update_estimate(float u, bool ball_removed, bool saturated) {
  if (ball_removed)
  {
    // Reset state estimate to zero
    z = {0.0, 0.0, 0.0, 0.0};
  }
  else
  {    
    float e_change_new = (error - e_last) / DT;    
   
    // Construct full state estimate
    // Put physical state in states 0, 1
    z(0) = mix(error, z(0), error_mix);
    z(1) = mix(e_change_new, z(1), error_diff_mix);
    
    // Put error sum in state 2, use anti-windup logic
    if (!saturated) {
      z(2) += error*DT;
    }

    // Put current control input in state 3
    z(3) = u;      

    // Remember variables for next time
    e_last = error;    
    Serial.println(e_last);
  }
  return z;
}

float PIDEstimator::get_error(){
  return error;
}

BLA::Matrix<4> PIDEstimator::get_estimate() {
  return z;
}
