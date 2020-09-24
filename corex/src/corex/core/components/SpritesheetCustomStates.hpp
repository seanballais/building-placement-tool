#ifndef COREX_CORE_COMPONENTS_SPRITESHEET_CUSTOM_STATES_HPP
#define COREX_CORE_COMPONENTS_SPRITESHEET_CUSTOM_STATES_HPP

#include <EASTL/string.h>
#include <EASTL/unordered_map.h>

#include <corex/core/asset_types/SpritesheetState.hpp>

namespace corex::core
{
  struct SpritesheetCustomStates
  {
    eastl::unordered_map<eastl::string, SpritesheetState> customStates;
  };
}

#endif
