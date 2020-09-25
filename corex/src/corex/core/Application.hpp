#ifndef COREX_CORE_APPLICATION_HPP
#define COREX_CORE_APPLICATION_HPP

#include <EASTL/string.h>
#include <EASTL/unique_ptr.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>
#include <SDL_gpu.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/DebugUI.hpp>
#include <corex/core/PerformanceMetrics.hpp>
#include <corex/core/SceneManager.hpp>
#include <corex/core/Settings.hpp>
#include <corex/core/WindowManager.hpp>
#include <corex/core/events/game_events.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/KeyboardHandler.hpp>
#include <corex/core/systems/MouseHandler.hpp>
#include <corex/core/systems/SpritesheetAnimation.hpp>
#include <corex/core/systems/SysEventDispatcher.hpp>

namespace corex::core
{
  class Application
  {
  public:
    Application(const eastl::string& windowTitle);
    virtual ~Application();

    virtual void init() = 0;
    virtual void dispose() = 0;

    void run();

  protected:
    SDL_GLContext glContext; // NOTE: SDL_GLContext is an alias for void*.

    // We're using unique pointers here to scene manager and asset manager
    // since we can't initialize them prior to execution of the constructor
    // body. We can define the necessary constructors in SceneManager and
    // AssetManager to allow them to be initialized without the objects
    // they need. But, using pointers to them is easier and is not much of
    // a hassle, which is the reason why we used the pointer-bassed approach.
    eastl::unique_ptr<SceneManager> sceneManager;
    eastl::unique_ptr<AssetManager> assetManager;

    entt::registry registry;
    entt::dispatcher eventDispatcher;
    SysEventDispatcher sysEventDispatcher;
    KeyboardHandler keyboardHandler;
    MouseHandler mouseHandler;
    SpritesheetAnimation spritesheetAnimation;
    Camera camera;
    DebugUI debugUI;
    Settings settings;
    WindowManager windowManager;
    float gameTimeWarpFactor;
    float isGamePlaying;
    eastl::string imGuiFilePath;

  private:
    void displayGraphicsAPIInfo();
    void runEventSystems();
    void dispatchPerformanceMetrics(PerformanceMetrics& metrics);
    void dispatchSceneManagerEvents();
    void computePerformanceMetrics(PerformanceMetrics& metrics);
    void handleGameTimeWarpEvents(const GameTimeWarpEvent& e);
    void handleGameTimerStatusEvents(const GameTimerStatusEvent& e);
    void handleWindowEvents(const WindowEvent& e);
    void renderPrep();
    void render();
  };
}

#endif
