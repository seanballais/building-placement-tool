#include <EASTL/string.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/asset_types/SpritesheetData.hpp>
#include <corex/core/asset_types/SpritesheetState.hpp>
#include <corex/core/asset_types/Texture.hpp>
#include <corex/core/components/Sprite.hpp>
#include <corex/core/components/SpritesheetCurrentFrameIndex.hpp>
#include <corex/core/components/SpritesheetCurrentState.hpp>
#include <corex/core/components/SpritesheetCustomStates.hpp>
#include <corex/core/components/SpritesheetFrameData.hpp>
#include <corex/core/components/SpritesheetFrames.hpp>

namespace corex::core
{
  void addTextureToEntity(entt::entity e, eastl::string&& textureID,
                          entt::registry& registry, AssetManager& assetManager,
                          bool reloadTexture)
  {
    Texture texture = assetManager.getTexture(textureID, reloadTexture);
    registry.emplace<Sprite>(e, 0, 0, texture->w, texture->h, texture);
  }

  void addSpritesheetToEntity(entt::entity e, eastl::string&& spritesheetID,
                              entt::registry& registry,
                              AssetManager& assetManager,
                              bool reloadSpritesheet)
  {
    SpritesheetData data = assetManager.getSpritesheet(spritesheetID,
                                                       reloadSpritesheet);
    registry.emplace<SpritesheetCurrentFrameIndex>(e, 0);
    registry.emplace<SpritesheetCustomStates>(e, data.customStates);
    registry.emplace<SpritesheetCurrentState>(e, data.customStates["all"]);
    registry.emplace<SpritesheetFrames>(e, data.frames);
    registry.emplace<SpritesheetFrameData>(e, data.timePerFrame, 0.0f);

    SDL_Rect currFrame = data.frames[0];
    registry.emplace<Sprite>(e,
                             currFrame.x,
                             currFrame.y,
                             currFrame.w,
                             currFrame.h,
                             data.image);
  }

  void setEntityAnimationState(entt::entity e, eastl::string&& state,
                               entt::registry& registry)
  {
    auto& currentFrameIndex = registry.get<SpritesheetCurrentFrameIndex>(e);
    auto& customStates = registry.get<SpritesheetCustomStates>(e);
    auto& currentState = registry.get<SpritesheetCurrentState>(e);
    auto& frames = registry.get<SpritesheetFrames>(e);
    auto& frameData = registry.get<SpritesheetFrameData>(e);
    auto& sprite = registry.get<Sprite>(e);

    SpritesheetState newState = customStates.customStates[state];
    currentState.currentState = newState;
    currentFrameIndex.currFrameIndex = currentState.currentState.startFrame;
    frameData.currFrameElapsedTime = 0.0f;

    SDL_Rect currFrame = frames.frames[currentFrameIndex.currFrameIndex];
    sprite.x = currFrame.x;
    sprite.y = currFrame.y;
    sprite.width = currFrame.w;
    sprite.height = currFrame.h;
  }
}
