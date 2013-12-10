#ifndef ROBTIMER_H
#define ROBTIMER_H
#include <ctime>
#include <sys/time.h>

class RobTimer
{
private:
  timeval _start;
public:
  //Start timer
  RobTimer();

  float getTime();

};

#endif // ROBTIMER_H
