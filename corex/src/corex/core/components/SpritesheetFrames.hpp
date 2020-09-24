#ifndef COREX_CORE_COMPONENTS_SPRITESHEET_FRAMES_HPP
#define COREX_CORE_COMPONENTS_SPRITESHEET_FRAMES_HPP

#include <EASTL/vector.h>
#include <SDL2/SDL.h>

namespace corex::core
{
  struct SpritesheetFrames
  {
    eastl::vector<SDL_Rect> frames;
  };
}

#endif
