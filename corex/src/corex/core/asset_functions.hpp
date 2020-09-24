#ifndef COREX_CORE_ASSET_FUNCTIONS_HPP
#define COREX_CORE_ASSET_FUNCTIONS_HPP

#include <EASTL/string.h>
#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/components/Sprite.hpp>

namespace corex::core
{
  void addTextureToEntity(entt::entity e, eastl::string&& textureID,
                          entt::registry& registry, AssetManager& assetManager,
                          bool reloadTexture = false);
  void addSpritesheetToEntity(entt::entity e, eastl::string&& spritesheetID,
                              entt::registry& registry,
                              AssetManager& assetManager,
                              bool reloadSpritesheet = false);
  void setEntityAnimationState(entt::entity e, eastl::string&& state,
                               entt::registry& registry);
}

#endif
