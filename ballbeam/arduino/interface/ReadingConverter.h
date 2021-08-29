#include <Servo.h>
#ifndef MY_READINGCONVERTER_H
#define MY_READINGCONVERTER_H

#include <Arduino.h>


class ReadingConverter {
  private:
    float observation;

  public:
    ReadingConverter();
    void update(int reading);
    float get_observation();
};

#endif
