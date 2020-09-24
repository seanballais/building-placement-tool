#include <cstdlib>

#include <entt/entt.hpp>

#include <EASTL/unique_ptr.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/ReturnState.hpp>
#include <corex/core/sdl_deleters.hpp>
#include <corex/core/Settings.hpp>
#include <corex/core/SettingsValue.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/WindowManager.hpp>
#include <corex/core/events/sys_events.hpp>

namespace corex::core
{
  WindowManager::WindowManager(const eastl::string& windowTitle,
                               entt::dispatcher& eventDispatcher,
                               Settings& settings)
  {
    GPU_SetPreInitFlags(GPU_INIT_DISABLE_VSYNC
                        | GPU_INIT_REQUEST_COMPATIBILITY_PROFILE);

    GPU_Target* renderTarget = nullptr;

    GPU_WindowFlagEnum windowFlags = SDL_WINDOW_OPENGL;
    uint32_t windowWidth = 800;
    uint32_t windowHeight = 600;

    auto windowWidthSettings = settings.getUnsignedIntVariable("window_width");
    auto windowHeightSettings = settings.getUnsignedIntVariable(
      "window_height");
    auto windowResizeSettings = settings.getBooleanVariable(
      "window_is_resizeable");

    if (windowWidthSettings.returnState == ReturnState::RETURN_OK) {
      windowWidth = windowWidthSettings.value;
    } else {
      STUBBED("Add message when settings does not have window_width setting.");
      settings.setVariable("window_width", 800);
    }

    if (windowHeightSettings.returnState == ReturnState::RETURN_OK) {
      windowWidth = windowHeightSettings.value;
    } else {
      STUBBED("Add message when settings does not have window_height setting.");
      settings.setVariable("window_height", 600);
    }

    if (windowResizeSettings.returnState == ReturnState::RETURN_OK) {
      windowFlags |= SDL_WINDOW_RESIZABLE;
    } else {
      STUBBED("Add message when settings does not "
              "have window_is_resizeable setting.");
      settings.setVariable("window_is_resizeable", true);
    }

    renderTarget = GPU_Init(windowWidth, windowHeight, windowFlags);

    SDL_Window* windowPtr = SDL_GetWindowFromID(renderTarget->context
                                                            ->windowID);
    SDL_SetWindowTitle(windowPtr, windowTitle.c_str());
    SDL_SetWindowPosition(windowPtr,
                          SDL_WINDOWPOS_CENTERED,
                          SDL_WINDOWPOS_CENTERED);

    this->glContext = renderTarget->context->context;
    this->window = eastl::unique_ptr<SDL_Window, SDLWindowDeleter>(windowPtr);
    this->renderTarget = eastl::unique_ptr<GPU_Target, SDLGPUTargetDeleter>(
      renderTarget
    );

    eventDispatcher.sink<WindowEvent>()
                   .connect<&WindowManager::handleWindowEvents>(this);

    settings.save();
  }

  WindowManager::~WindowManager()
  {
    this->renderTarget.release();
    GPU_Quit();

    SDL_Window* windowPtr = this->window.release();
    SDL_DestroyWindow(windowPtr);
  }

  SDL_Window* WindowManager::getWindow()
  {
    return this->window.get();
  }

  GPU_Target* WindowManager::getRenderTarget()
  {
    return this->renderTarget.get();
  }

  SDL_GLContext WindowManager::getOpenGLContext()
  {
    return this->renderTarget->context->context;
  }

  void WindowManager::handleWindowEvents(const WindowEvent& e)
  {
    switch (e.event.window.event) {
      case SDL_WINDOWEVENT_RESIZED:
        GPU_SetWindowResolution(e.event.window.data1, e.event.window.data2);
        break;
    }
  }
}
