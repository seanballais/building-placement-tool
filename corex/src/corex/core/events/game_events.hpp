#ifndef COREX_CORE_EVENTS_GAME_EVENTS_HPP
#define COREX_CORE_EVENTS_GAME_EVENTS_HPP

namespace corex::core
{
  struct GameTimeWarpEvent
  {
    float timeWarpFactor;
  };

  struct GameTimerStatusEvent
  {
    bool isPlaying;
  };
}

#endif
