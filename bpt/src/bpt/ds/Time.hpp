#ifndef BPT_DS_TIME_HPP
#define BPT_DS_TIME_HPP

#include <cstdint>

struct Time
{
  Time(double timeInSeconds);

  int32_t hours;
  int32_t minutes;
  int32_t seconds;
};

#endif
