#ifndef COREX_EVENTS_KB_EVENTS_HPP
#define COREX_EVENTS_KB_EVENTS_HPP

#include <cstdlib>

#include <SDL2/SDL.h>

#include <corex/core/systems/KeyState.hpp>

namespace corex::core
{
  struct KeyboardEvent
  {
    SDL_Keycode keyCode;
    KeyState keyState;
    int32_t numRepeats;
  };
}

#endif
