#ifndef COREX_CORE_COMPONENTS_RENDER_RECTANGLE_HPP
#define COREX_CORE_COMPONENTS_RENDER_RECTANGLE_HPP

#include <SDL2/SDL.h>

namespace corex::core
{
  struct RenderRectangle
  {
    // x and y will refer to the center of the render rectangle.
    float x;
    float y;
    float width;
    float height;
    float angle; // Expected to be in degrees.
    SDL_Color colour;
    bool isFilled;
  };
}

#endif
