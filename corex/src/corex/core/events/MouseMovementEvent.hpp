#ifndef COREX_EVENTS_MOUSE_MOVEMENT_EVENT_HPP
#define COREX_EVENTS_MOUSE_MOVEMENT_EVENT_HPP

#include <cstdlib>

namespace corex::core
{
  struct MouseMovementEvent
  {
    int32_t x;
    int32_t y;
    int32_t xDelta;
    int32_t yDelta;
  };
}

#endif
