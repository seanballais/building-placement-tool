#ifndef COREX_CORE_SYSTEMS_MOUSE_HANDLER_HPP
#define COREX_CORE_SYSTEMS_MOUSE_HANDLER_HPP

#include <cstdlib>

#include <EASTL/array.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/MouseButtonState.hpp>

namespace corex::core
{
  class MouseHandler : public BaseSystem
  {
    // Currently, we're going to assume that only one mouse is used.
  public:
    MouseHandler(entt::dispatcher& eventDispatcher);

    void handleMouseButtonDowns(const MouseButtonDownEvent& e);
    void handleMouseButtonUps(const MouseButtonUpEvent& e);
    void handleMouseMovements(const MouseMotionEvent& e);
    void handleMouseScrolls(const MouseWheelEvent& e);
    void update();

  private:
    eastl::array<MouseButtonState, 3> mouseButtonStates;
    eastl::array<int32_t, 3> mouseButtonNumRepeats;
    int32_t x;
    int32_t y;
    int32_t xMovementDelta;
    int32_t yMovementDelta;
    int32_t xScrollAmount;
    int32_t yScrollAmount;
  };
}

#endif
