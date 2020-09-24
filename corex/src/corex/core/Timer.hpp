#ifndef COREX_CORE_TIMER_HPP
#define COREX_CORE_TIMER_HPP

#include <cstdint>

namespace corex::core
{
  class Timer
  {
  public:
    Timer();

    void start();
    void pause();
    void resume();
    void stop();
    void reset();
    double getElapsedTime();

  private:
    uint64_t startTime;
    bool isRunning;
    bool isStopped;
  };
}

#endif
