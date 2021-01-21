#include <cmath>
#include <cstdint>

#include <bpt/ds/Time.hpp>

Time::Time(double timeInSeconds)
  : hours(0)
  , minutes(0)
  , seconds(0)
{
  this->hours = timeInSeconds / 3600;
  timeInSeconds = std::fmod(timeInSeconds, 3600.0);

  this->minutes = timeInSeconds / 60;
  timeInSeconds = std::fmod(timeInSeconds, 60.0);

  this->seconds = static_cast<int32_t>(std::round(timeInSeconds));
}
