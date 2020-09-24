#ifndef COREX_CORE_ASSET_DB_RECORD_HPP
#define COREX_CORE_ASSET_DB_RECORD_HPP

#include <cstdint>

#include <EASTL/string.h>

#include <corex/core/Asset.hpp>

namespace corex::core
{
  struct AssetDBRecord
  {
    Asset data;
    eastl::string assetType;
    eastl::string filePath;
  };
}

#endif
