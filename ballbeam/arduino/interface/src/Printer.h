#ifndef MY_PRINTER_H
#define MY_PRINTER_H

#include <Arduino.h>

class Printer {
private:
    bool mask[8];
    char header_strings[8][40];
    char spacer;

public:
    Printer();

    void print_header();
    void print(float setpoint, float error, BLA::Matrix<4> state_estimate, float action, unsigned int servo_cmd, unsigned long time_change);    
};

#endif
