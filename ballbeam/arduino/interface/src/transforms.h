#ifndef MY_TRANSFORMS_H
#define MY_TRANSFORMS_H

#include <Arduino.h>

float mix(float a, float b, float x);
float soft(float a, float x);
float polyval5(float c[], float x);
float reading2observation(int reading);
long action2actuation(float u);

#endif
