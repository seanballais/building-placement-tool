#ifndef BOYET_SCENES_THE_VOID_CPP
#define BOYET_SCENES_THE_VOID_CPP

#include <cstdlib>

#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/events/sys_events.hpp>

namespace boyet::scenes
{
  class TheVoid : public corex::core::Scene
  {
  public:
    TheVoid(entt::registry& registry,
            entt::dispatcher& eventDispatcher,
            corex::core::AssetManager& assetManager);

    void init() override;
    void update(float timeDelta) override;
    void dispose() override;

  private:
    void handleWindowEvents(const corex::core::WindowEvent& e);

    entt::entity rectangle0Entity;
    entt::entity rectangle1Entity;
    entt::entity intersectionEntity;
    int32_t numIntersections;
  };
}

#endif
