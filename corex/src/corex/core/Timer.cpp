#include <cstdint>

#include <SDL2/SDL.h>

#include <corex/core/Timer.hpp>

namespace corex::core
{
  Timer::Timer()
    : startTime(SDL_GetPerformanceCounter())
    , isRunning(false)
    , isStopped(true) {}

  void Timer::start()
  {
    this->isRunning = true;

    if (this->isStopped) {
      startTime = SDL_GetPerformanceCounter();
      this->isStopped = false;
    }
  }

  void Timer::pause()
  {
    this->isRunning = false;
  }

  void Timer::resume()
  {
    this->startTime = SDL_GetPerformanceCounter() - this->startTime;
    this->isRunning = true;
  }

  void Timer::stop()
  {
    this->isRunning = false;
    this->isStopped = true;
  }

  void Timer::reset()
  {
    this->stop();
    this->start();
  }

  double Timer::getElapsedTime() const
  {
    uint64_t timeNow = SDL_GetPerformanceCounter();

    // Get elapsed time in seconds. Code based from here:
    //   https://gamedev.stackexchange.com/a/110831/93942
    return static_cast<double>(
      static_cast<double>(timeNow - this->startTime)
      / static_cast<double>(SDL_GetPerformanceFrequency())
    );
  }
}
