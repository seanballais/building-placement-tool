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
    [[nodiscard]] double getElapsedTime() const;

  private:
    uint64_t startTime;
    bool isRunning;
    bool isStopped;
  };
}

namespace cx
{
  using namespace corex::core;
}

#endif
