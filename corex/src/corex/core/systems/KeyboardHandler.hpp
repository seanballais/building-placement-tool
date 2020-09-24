#ifndef COREX_CORE_SYSTEMS_KEYBOARD_HANDLER_HPP
#define COREX_CORE_SYSTEMS_KEYBOARD_HANDLER_HPP

#include <cstdlib>

#include <EASTL/unordered_map.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/KeyStatus.hpp>

namespace corex::core
{
  class KeyboardHandler : public BaseSystem
  {
  public:
    KeyboardHandler(entt::dispatcher& eventDispatcher);

    void handleKeyUps(const KeyUpEvent& e);
    void handleKeyDowns(const KeyDownEvent& e);
    void update();

  private:
    eastl::unordered_map<SDL_Keycode, KeyStatus> keyStatuses;
  };
}

#endif
