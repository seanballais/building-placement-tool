#ifndef COREX_CORE_ASSET_TYPES_TEXTURE_HPP
#define COREX_CORE_ASSET_TYPES_TEXTURE_HPP

#include <EASTL/shared_ptr.h>
#include <SDL_gpu.h>

namespace corex::core
{
  // We're using a shared pointer here so that we can safely reload the
  // asset database in real time without worrying about invalidating
  // pre-existing pointers to the old data. This is useful during development
  // when we want to see changes in the asset database in real time.
  using Texture = eastl::shared_ptr<GPU_Image>;
}

#endif
