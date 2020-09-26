#ifndef COREX_CORE_COMPONENTS_RENDER_LINE_SEGMENTS_HPP
#define COREX_CORE_COMPONENTS_RENDER_LINE_SEGMENTS_HPP

#include <EASTL/vector.h>
#include <SDL2/SDL.h>

#include <corex/core/ds/Point.hpp>

namespace corex::core
{
  struct RenderLineSegments
  {
    eastl::vector<Point> vertices;
    SDL_Color colour;
  };
}

#endif
