#ifndef COREX_CORE_COMPONENTS_SPRITESHEET_FRAME_DATA_HPP
#define COREX_CORE_COMPONENTS_SPRITESHEET_FRAME_DATA_HPP

#include <EASTL/shared_ptr.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

namespace corex::core
{
  struct SpritesheetFrameData
  {
    float timePerFrame;
    float currFrameElapsedTime;
  };
}

#endif
