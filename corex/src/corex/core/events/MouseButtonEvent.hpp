#ifndef COREX_EVENTS_MOUSE_BUTTON_EVENT_HPP
#define COREX_EVENTS_MOUSE_BUTTON_EVENT_HPP

#include <cstdlib>

#include <corex/core/systems/MouseButtonState.hpp>
#include <corex/core/systems/MouseButtonType.hpp>

namespace corex::core
{
  struct MouseButtonEvent
  {
    MouseButtonType buttonType;
    MouseButtonState buttonState;
  };
}

#endif
