#ifndef MY_ESTIMATOR_H
#define MY_ESTIMATOR_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>


// Linear state estimator class
class LinearEstimator {
  private:
    BLA::Matrix<2, 2> A;
    BLA::Matrix<2, 1> B;
    BLA::Matrix<1, 2> C;
    BLA::Matrix<2, 1> L;
    BLA::Matrix<2, 2> AL;
    BLA::Matrix<2> x;
    BLA::Matrix<4> z;
    BLA::Matrix<1> y;
    float error;
    float DT;

  public:
    LinearEstimator(BLA::Matrix<2, 2> A, BLA::Matrix<2, 1> B, BLA::Matrix<1, 2> C, BLA::Matrix<2, 1> L, float DT);
    float update_error(float observation, float setpoint);
    BLA::Matrix<4> update_estimate(float u, bool ball_removed, bool saturated);
    float get_error();
    BLA::Matrix<4> get_estimate();
};


class PIDEstimator {
  private:
    float error_mix;
    float error_diff_mix;
    float error;
    float DT;
    float e_last;
    BLA::Matrix<4> z;

  public:
    PIDEstimator(float error_mix, float error_diff_mix, float DT);
    float update_error(float observation, float setpoint);
    BLA::Matrix<4> update_estimate(float u, bool ball_removed, bool saturated);
    float get_error();
    BLA::Matrix<4> get_estimate();
};

#endif
