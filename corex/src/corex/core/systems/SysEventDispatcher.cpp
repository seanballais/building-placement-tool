#include <iostream>
#include <string>

#include <entt/entt.hpp>
#include <imgui.h>
#include <imgui_impls/imgui_impl_sdl.h>
#include <SDL2/SDL.h>

#include <corex/core/utils.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/SysEventDispatcher.hpp>

namespace corex::core
{
  SysEventDispatcher::SysEventDispatcher(entt::dispatcher& eventDispatcher)
    : BaseSystem(eventDispatcher) {}

  void SysEventDispatcher::update()
  {
    ImGuiIO& io = ImGui::GetIO();
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (io.WantCaptureKeyboard || io.WantCaptureMouse || io.WantTextInput) {
        continue;
      }

      switch (event.type) {
        case SDL_WINDOWEVENT:
          this->eventDispatcher.trigger<WindowEvent>(event);
          break;
        case SDL_KEYDOWN:
          this->eventDispatcher.trigger<KeyDownEvent>(event);
          break;
        case SDL_KEYUP:
          this->eventDispatcher.trigger<KeyUpEvent>(event);
          break;
        case SDL_MOUSEBUTTONDOWN:
          this->eventDispatcher.trigger<MouseButtonDownEvent>(event);
          break;
        case SDL_MOUSEBUTTONUP:
          this->eventDispatcher.trigger<MouseButtonUpEvent>(event);
          break;
        case SDL_MOUSEMOTION:
          this->eventDispatcher.trigger<MouseMotionEvent>(event);
          break;
        case SDL_MOUSEWHEEL:
          this->eventDispatcher.trigger<MouseWheelEvent>(event);
          break;
        default:
          STUBBED("Unsupported SDL event type: " + std::to_string(event.type));
      }
    }
  }
}
