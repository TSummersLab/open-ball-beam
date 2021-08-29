#include <Servo.h>
#include "ReadingConverter.h"
#include "transforms.h"


ReadingConverter::ReadingConverter() {
  this->observation = 0.0;
}

void ReadingConverter::update(int reading) {
  // Convert raw reading to standard observation
  observation = reading2observation(reading);
}

float ReadingConverter::get_observation() {
  return observation;
}
