#ifndef COREX_CORE_COMPONENTS_SPRITE_HPP
#define COREX_CORE_COMPONENTS_SPRITE_HPP

#include <cstdint>

#include <EASTL/shared_ptr.h>
#include <SDL_gpu.h>

namespace corex::core
{
  struct Sprite
  {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    eastl::shared_ptr<GPU_Image> texture;
  };
}

#endif
