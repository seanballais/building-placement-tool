#include <cstdlib>

#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/MouseScrollEvent.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/MouseButtonState.hpp>
#include <corex/core/systems/MouseButtonType.hpp>
#include <corex/core/systems/MouseHandler.hpp>

namespace corex::core
{
  MouseHandler::MouseHandler(entt::dispatcher& eventDispatcher)
    : BaseSystem(eventDispatcher)
    , mouseButtonStates()
  {
    this->eventDispatcher.sink<MouseButtonDownEvent>()
                         .connect<&MouseHandler::handleMouseButtonDowns>(this);
    this->eventDispatcher.sink<MouseButtonUpEvent>()
                         .connect<&MouseHandler::handleMouseButtonUps>(this);
    this->eventDispatcher.sink<MouseMotionEvent>()
                         .connect<&MouseHandler::handleMouseMovements>(this);
    this->eventDispatcher.sink<MouseWheelEvent>()
                         .connect<&MouseHandler::handleMouseScrolls>(this);
  }

  void MouseHandler::handleMouseButtonDowns(const MouseButtonDownEvent& e)
  {
    int32_t buttonIndex = 0;
    switch (e.event.button.button) {
      case SDL_BUTTON_LEFT:
        buttonIndex = 0;
        break;
      case SDL_BUTTON_MIDDLE:
        buttonIndex = 1;
        break;
      case SDL_BUTTON_RIGHT:
        buttonIndex = 2;
        break;
    }

    this->mouseButtonStates[buttonIndex] = MouseButtonState::MOUSE_BUTTON_DOWN;
  }

  void MouseHandler::handleMouseButtonUps(const MouseButtonUpEvent& e)
  {
    int32_t buttonIndex = 0;
    switch (e.event.button.button) {
      case SDL_BUTTON_LEFT:
        buttonIndex = 0;
        break;
      case SDL_BUTTON_MIDDLE:
        buttonIndex = 1;
        break;
      case SDL_BUTTON_RIGHT:
        buttonIndex = 2;
        break;
    }

    this->mouseButtonStates[buttonIndex] = MouseButtonState::MOUSE_BUTTON_UP;
  }

  void MouseHandler::handleMouseMovements(const MouseMotionEvent& e)
  {
    this->xMovementDelta = e.event.motion.xrel;
    this->yMovementDelta = e.event.motion.yrel;
  }

  void MouseHandler::handleMouseScrolls(const MouseWheelEvent& e)
  {
    this->xScrollAmount = e.event.wheel.x;
    this->yScrollAmount = e.event.wheel.y;
  }

  void MouseHandler::update()
  {
    this->eventDispatcher.enqueue<MouseButtonEvent>(
      MouseButtonType::MOUSE_BUTTON_LEFT,
      this->mouseButtonStates[0]);
    this->eventDispatcher.enqueue<MouseButtonEvent>(
      MouseButtonType::MOUSE_BUTTON_MIDDLE,
      this->mouseButtonStates[1]);
    this->eventDispatcher.enqueue<MouseButtonEvent>(
      MouseButtonType::MOUSE_BUTTON_RIGHT,
      this->mouseButtonStates[2]);
    this->eventDispatcher.enqueue<MouseMovementEvent>(this->xMovementDelta,
                                                      this->yMovementDelta);
    this->eventDispatcher.enqueue<MouseScrollEvent>(this->xScrollAmount,
                                                    this->yScrollAmount);

    // Reset the following values, so that they won't have any effect later,
    // in the case that the mouse wasn't moved or the wheel scrolled.
    this->xMovementDelta = 0;
    this->yMovementDelta = 0;
    this->xScrollAmount = 0;
    this->yScrollAmount = 0;
  }
}
