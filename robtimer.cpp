#include "robtimer.h"

RobTimer::RobTimer()
{
  gettimeofday(&_start, NULL);
}

float RobTimer::getTime(){
  timeval stop;
  gettimeofday(&stop, NULL);
  float delta_t  = (stop.tv_sec - _start.tv_sec) * 1000.0;               // sec to ms
  delta_t += (stop.tv_usec - _start.tv_usec) / 1000.0;            // us to ms
  delta_t  = delta_t / 1000.0;
  return delta_t;
}
