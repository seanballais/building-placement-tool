#ifndef COREX_CORE_ASSET_TYPES_SPRITESHEET_STATE_HPP
#define COREX_CORE_ASSET_TYPES_SPRITESHEET_STATE_HPP

#include <cstdlib>

namespace corex::core
{
  struct SpritesheetState
  {
    int32_t startFrame;
    int32_t endFrame;
    bool isLooping;
  };
}

#endif
