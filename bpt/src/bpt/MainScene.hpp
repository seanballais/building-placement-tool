#ifndef BPT_MAIN_SCENE_HPP
#define BPT_MAIN_SCENE_HPP

#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/ds/Circle.hpp>
#include <corex/core/ds/LineSegments.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/sys_events.hpp>

#include <bpt/Context.hpp>

namespace bpt
{
  class MainScene : public corex::core::Scene
  {
  public:
    MainScene(entt::registry& registry,
              entt::dispatcher& eventDispatcher,
              corex::core::AssetManager& assetManager);

    void init() override;
    void update(float timeDelta) override;
    void dispose() override;

  private:
    void buildConstructBoundingAreaWindow();
    void handleWindowEvents(const corex::core::WindowEvent& e);
    void handleMouseButtonEvents(const corex::core::MouseButtonEvent& e);
    void handleMouseMovementEvents(const corex::core::MouseMovementEvent& e);

    Context currentContext;
    bool isCloseAreaTriggerEnabled;
    corex::core::Circle closeAreaTriggerCircle;
    corex::core::LineSegments wipBoundingArea;
    corex::core::NPolygon boundingArea;
    entt::entity wipBoundingAreaEntity;
    entt::entity boundingAreaEntity;
    entt::entity closeAreaTriggerEntity;
  };
}

#endif
