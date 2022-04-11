#ifndef MY_BALL_H
#define MY_BALL_H

#include <Arduino.h>


// Class to handle the removed state of the ball
class Ball {
  private:
    bool removed;    
    float backstop;
    long count;
    long timeout_count;
  
  public:
    Ball();
    Ball(bool removed, float backstop, long timeout_count);
    long get_count();
    bool is_removed();
    bool update_removed_status(float obs);
};

#endif
