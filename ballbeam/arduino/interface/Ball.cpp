#include "Ball.h"


Ball::Ball(bool removed, float backstop, long timeout_count) {
  this->removed = removed;
  this->backstop = backstop;
  this->count = 0;
  this->timeout_count = timeout_count;
}

void Ball::update(float obs) {    
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
}

int Ball::get_count() {
  return count;
}

bool Ball::is_removed() {
  return (count > timeout_count);
}    
