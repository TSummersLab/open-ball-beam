#include <Servo.h>
#ifndef MY_ISSUER_H
#define MY_ISSUER_H

#include <Arduino.h>


class CommandIssuer {
  private:
    Servo servo;
    unsigned int servo_cmd;
    unsigned int servo_cmd_mid;
    unsigned int servo_cmd_min;
    unsigned int servo_cmd_max;
    bool saturated;

  public:
    CommandIssuer(Servo servo);
    CommandIssuer(Servo servo, unsigned int servo_cmd_mid, unsigned int servo_cmd_min, unsigned int servo_cmd_max);
    void issue_control(float u);
    unsigned int get_command();
    bool get_saturated();
};

#endif
