#ifndef COREX_SYSTEMS_CORE_SPRITESHEET_ANIMATION_HPP
#define COREX_SYSTEMS_CORE_SPRITESHEET_ANIMATION_HPP

#include <entt/entt.hpp>

#include <corex/core/events/metric_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>

namespace corex::core
{
  class SpritesheetAnimation : public BaseSystem
  {
  public:
    SpritesheetAnimation(entt::dispatcher& eventDispatcher,
                         entt::registry& registry);

    void update();

  private:
    void handleFrameDataEvents(const FrameDataEvent& e);

    entt::registry& registry;
    float timeDelta;
  };
}

#endif
