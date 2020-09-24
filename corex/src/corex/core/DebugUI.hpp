#ifndef COREX_CORE_DEBUG_UI_HPP
#define COREX_CORE_DEBUG_UI_HPP

#include <entt/entt.hpp>
#include <imgui.h>

#include <corex/core/Camera.hpp>
#include <corex/core/PerformanceMetrics.hpp>
#include <corex/core/events/game_events.hpp>
#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/metric_events.hpp>
#include <corex/core/events/scene_manager_events.hpp>
#include <corex/core/events/sys_events.hpp>

namespace corex::core
{
  class DebugUI
  {
  public:
    DebugUI(entt::dispatcher& eventDispatcher, Camera& camera);

    void update();
    void render();

  private:
    void buildMenu();
    void buildPerformanceMetrics();
    void buildCameraControls();
    void buildTimeControls();
    void dispatchTimeWarpEvents();
    void dispatchGameTimerStatusEvents();
    void handleFrameDataEvents(const FrameDataEvent& e);
    void handleKeyboardEvents(const KeyboardEvent& e);
    void handleMouseWheelEvents(const MouseWheelEvent& e);
    void handlePPMRatioChangeEvents(const PPMRatioChange& e);
    void handleGameTimeWarpEvents(const GameTimeWarpEvent& e);

    entt::dispatcher& eventDispatcher;
    PerformanceMetrics metrics;
    Camera& camera;
    bool isFreeFlyEnabled;
    bool isDebugUIDisplayed;
    bool isCameraControlsDisplayed;
    bool isPerformanceMetricsDisplayed;
    bool isTimeControlsDisplayed;
    bool isGamePlaying;
    float cameraZoomXDelta;
    float cameraZoomYDelta;
    float cameraMovementSpeed;
    float cameraAngle;
    float ppmRatio;
    float gameTimeWarpFactor;
  };
}

#endif
