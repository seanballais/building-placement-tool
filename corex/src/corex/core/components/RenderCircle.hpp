#ifndef COREX_CORE_COMPONENTS_RENDER_CIRCLE_HPP
#define COREX_CORE_COMPONENTS_RENDER_CIRCLE_HPP

#include <cstdlib>

#include <SDL2/SDL.h>

#include <corex/core/ds/Point.hpp>

namespace corex::core
{
  struct RenderCircle
  {
    float radius;
    SDL_Color colour;
    bool isFilled;
  };
}

#endif
