#include <Servo.h>
#include "ReadingConverter.h"
#include "transforms.h"


ReadingConverter::ReadingConverter() {
  this->observation = 0.0;
}

float ReadingConverter::update_observation(int reading) {
  // Convert raw reading to standard observation
  observation = reading2observation(reading);
  return observation;
}

float ReadingConverter::get_observation() {
  return observation;
}
