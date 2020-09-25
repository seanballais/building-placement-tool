#ifndef COREX_EVENTS_MOUSE_SCROLL_EVENT_HPP
#define COREX_EVENTS_MOUSE_SCROLL_EVENT_HPP

#include <cstdlib>

namespace corex::core
{
  struct MouseScrollEvent
  {
    int32_t xScrollAmount;
    int32_t yScrollAmount;
  };
}

#endif
