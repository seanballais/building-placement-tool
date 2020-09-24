#ifndef COREX_CORE_ASSET_TYPES_SPRITESHEET_DATA_HPP
#define COREX_CORE_ASSET_TYPES_SPRITESHEET_DATA_HPP

#include <cstdlib>

#include <EASTL/string.h>
#include <EASTL/shared_ptr.h>
#include <EASTL/unordered_map.h>
#include <EASTL/vector.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/asset_types/SpritesheetState.hpp>

namespace corex::core
{
  struct SpritesheetData
  {
    eastl::shared_ptr<GPU_Image> image;
    float timePerFrame;
    eastl::vector<SDL_Rect> frames;
    eastl::unordered_map<eastl::string, SpritesheetState> customStates;
  };
}

#endif
