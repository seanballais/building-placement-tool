#ifndef COREX_CORE_WINDOW_MANAGER_HPP
#define COREX_CORE_WINDOW_MANAGER_HPP

#include <entt/entt.hpp>

#include <EASTL/string.h>
#include <EASTL/unique_ptr.h>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/sdl_deleters.hpp>
#include <corex/core/Settings.hpp>
#include <corex/core/events/sys_events.hpp>

namespace corex::core
{
	class WindowManager
	{
	public:
		WindowManager(const eastl::string& windowTitle,
                  entt::dispatcher& eventDispatcher,
                  Settings& settings);
    ~WindowManager();

		SDL_Window* getWindow();
		GPU_Target* getRenderTarget();
    SDL_GLContext getOpenGLContext();
	private:
		void handleWindowEvents(const WindowEvent& e);

		eastl::unique_ptr<SDL_Window, SDLWindowDeleter> window;
    eastl::unique_ptr<GPU_Target, SDLGPUTargetDeleter> renderTarget;
    SDL_GLContext glContext;
	};
}

#endif
