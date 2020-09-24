#ifndef COREX_CORE_SYSTEMS_KEY_STATUS_HPP
#define COREX_CORE_SYSTEMS_KEY_STATUS_HPP

#include <cstdlib>

#include <SDL2/SDL.h>

#include <corex/core/systems/KeyState.hpp>

namespace corex::core
{
  struct KeyStatus
  {
    KeyState state;
    int32_t numRepeats;
  };
}

#endif
