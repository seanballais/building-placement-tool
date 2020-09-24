#ifndef COREX_SYSTEMS_CORE_BASE_SYSTEM_HPP
#define COREX_SYSTEMS_CORE_BASE_SYSTEM_HPP

#include <entt/entt.hpp>

namespace corex::core
{
  class BaseSystem
  {
  public:
    BaseSystem(entt::dispatcher& eventDispatcher);

  protected:
    entt::dispatcher& eventDispatcher;
  };
}

#endif
