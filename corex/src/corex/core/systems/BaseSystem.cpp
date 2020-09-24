#include <entt/entt.hpp>

#include <corex/core/systems/BaseSystem.hpp>

namespace corex::core
{
  BaseSystem::BaseSystem(entt::dispatcher& eventDispatcher)
    : eventDispatcher(eventDispatcher) {}
}
