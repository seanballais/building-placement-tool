#include <cstdlib>

#include <EASTL/array.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/KeyboardHandler.hpp>
#include <corex/core/systems/KeyState.hpp>

namespace corex::core
{
  KeyboardHandler::KeyboardHandler(entt::dispatcher& eventDispatcher)
    : BaseSystem(eventDispatcher)
    , keyStatuses()
  {
    this->eventDispatcher.sink<KeyUpEvent>()
                         .connect<&KeyboardHandler::handleKeyUps>(this);
    this->eventDispatcher.sink<KeyDownEvent>()
                         .connect<&KeyboardHandler::handleKeyDowns>(this);
  }

  void KeyboardHandler::handleKeyUps(const KeyUpEvent& e)
  {
    SDL_Keycode keyCode = e.event.key.keysym.sym;
    KeyState newState = KeyState::KEY_UP;
    int32_t numRepeats = 0;

    auto iter = this->keyStatuses.find(keyCode);
    if (iter != this->keyStatuses.end() && iter->second.state == newState) {
      // Kinda unnecessary in general, but it may be useful to keep track of the
      // number of key up repeats.
      numRepeats = iter->second.numRepeats + 1;
    }

    this->keyStatuses.insert_or_assign(keyCode,
                                       KeyStatus{newState, numRepeats});
  }

  void KeyboardHandler::handleKeyDowns(const KeyDownEvent& e)
  {
    SDL_Keycode keyCode = e.event.key.keysym.sym;
    KeyState newState = KeyState::KEY_DOWN;
    int32_t numRepeats = 0;

    auto iter = this->keyStatuses.find(keyCode);
    if (iter != this->keyStatuses.end() && iter->second.state == newState) {
      // Sometimes, a key down event is triggered even if the corresponding
      // key is still being pressed.
      numRepeats = iter->second.numRepeats + 1;
    }

    this->keyStatuses.insert_or_assign(keyCode,
                                       KeyStatus{newState, numRepeats});
  }

  void KeyboardHandler::update()
  {
    for (auto& pair : this->keyStatuses) {
      this->eventDispatcher.enqueue<KeyboardEvent>(pair.first,
                                                   pair.second.state,
                                                   pair.second.numRepeats);
      pair.second.numRepeats++;
    }
  }
}
