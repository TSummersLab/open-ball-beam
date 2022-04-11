#include <Servo.h>
#ifndef MY_READINGCONVERTER_H
#define MY_READINGCONVERTER_H

#include <Arduino.h>


class ReadingConverter {
  private:
    float observation;

  public:
    ReadingConverter();
    float get_observation();
    float update_observation(int reading);
};

#endif
