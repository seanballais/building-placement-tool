#ifndef COREX_CORE_ASSETS_ASSET_HPP
#define COREX_CORE_ASSETS_ASSET_HPP

#include <EASTL/variant.h>

#include <corex/core/asset_types/Font.hpp>
#include <corex/core/asset_types/SpritesheetData.hpp>
#include <corex/core/asset_types/Texture.hpp>

namespace corex::core
{
  using Asset = eastl::variant<
    eastl::monostate,
    Font,
    Texture,
    SpritesheetData>;
}

#endif
