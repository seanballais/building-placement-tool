#ifndef COREX_CORE_ASSET_MANAGER_HPP
#define COREX_CORE_ASSET_MANAGER_HPP

#include <EASTL/string.h>
#include <EASTL/unordered_map.h>
#include <SDL_gpu.h>

#include <corex/core/AssetDBRecord.hpp>
#include <corex/core/asset_types/Font.hpp>
#include <corex/core/asset_types/SpritesheetData.hpp>
#include <corex/core/asset_types/Texture.hpp>

namespace corex::core
{
  class AssetManager
  {
  public:
    AssetManager();

    Font getFont(eastl::string assetID, int32_t size, bool forceReload = false);
    Texture getTexture(eastl::string assetID, bool forceReload = false);
    SpritesheetData
    getSpritesheet(eastl::string assetID, bool forceReload = false);

    void unloadAsset(eastl::string assetID);
    void reloadAssetDB();

  private:
    void loadAssetDB();
    GPU_Image* loadTexture(eastl::string texturePath);

    eastl::unordered_map<eastl::string, AssetDBRecord> assetDB;
  };
}

#endif
