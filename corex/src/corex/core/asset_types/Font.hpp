#ifndef COREX_CORE_ASSET_TYPES_FONT_HPP
#define COREX_CORE_ASSET_TYPES_FONT_HPP

#include <EASTL/shared_ptr.h>
#include <SDL2/SDL_ttf.h>

namespace corex::core
{
  // We're using a shared pointer here so that we can safely reload the
  // asset database in real time without worrying about invalidating
  // pre-existing pointers to the old data. This is useful during development
  // when we want to see changes in the asset database in real time.
  // TODO: Switch to using unique_ptr during production. Maybe create
  //       a custom asset_ptr too.
  using Font = eastl::shared_ptr<TTF_Font>;
}

#endif
