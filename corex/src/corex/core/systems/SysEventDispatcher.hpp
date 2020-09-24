#ifndef COREX_SYSTEMS_CORE_SYS_EVENTS_DISPATCHER_HPP
#define COREX_SYSTEMS_CORE_SYS_EVENTS_DISPATCHER_HPP

#include <entt/entt.hpp>

#include <corex/core/systems/BaseSystem.hpp>

namespace corex::core
{
  class SysEventDispatcher : public BaseSystem
  {
  public:
    SysEventDispatcher(entt::dispatcher& eventDispatcher);

    void update();
  };
}

#endif
