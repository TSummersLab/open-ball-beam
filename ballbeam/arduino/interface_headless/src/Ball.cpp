#include "Ball.h"


extern const long TIMEOUT_COUNT;
extern const float OBS_BACKSTOP;

Ball::Ball() {
    this->removed = false;
    this->backstop = OBS_BACKSTOP;
    this->count = 0;
    this->timeout_count = TIMEOUT_COUNT;
}

Ball::Ball(bool removed, float backstop, long timeout_count) {
  this->removed = removed;
  this->backstop = backstop;
  this->count = 0;
  this->timeout_count = timeout_count;
}

bool Ball::update_removed_status(float obs) {
  if (!removed) {
    if (obs > backstop) {
      count += 1;
    }
    else {
      count = 0;
    }
  }
  else {
    if (obs <= backstop) {
      count -= 1;
    }
    else {
      count = 2 * timeout_count;
    }
  } 
  return (count > timeout_count);
}

long Ball::get_count() {
  return count;
}

bool Ball::is_removed() {
  return (count > timeout_count);
}    
