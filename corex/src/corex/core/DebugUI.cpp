#include <EASTL/string.h>
#include <entt/entt.hpp>
#include <imgui.h>
#include <imgui_impls/imgui_impl_opengl3.h>
#include <imgui_impls/imgui_impl_sdl.h>

#include <corex/core/DebugUI.hpp>
#include <corex/core/PerformanceMetrics.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/events/game_events.hpp>
#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/metric_events.hpp>
#include <corex/core/events/scene_manager_events.hpp>
#include <corex/core/events/sys_events.hpp>

#include <iostream>

namespace corex::core
{
  DebugUI::DebugUI(entt::dispatcher& eventDispatcher, Camera& camera)
    : eventDispatcher(eventDispatcher)
    , metrics()
    , camera(camera)
    , isFreeFlyEnabled(false)
    , isDebugUIDisplayed(false)
    , isCameraControlsDisplayed(false)
    , isPerformanceMetricsDisplayed(false)
    , isTimeControlsDisplayed(false)
    , isGamePlaying(true)
    , cameraZoomXDelta(0.15f)
    , cameraZoomYDelta(0.15f)
    , cameraMovementSpeed(10.0f)
    , cameraAngle(0.0f)
    , ppmRatio(0.0f)
    , gameTimeWarpFactor(1.0f)
  {
    this->eventDispatcher.sink<FrameDataEvent>()
                         .connect<&DebugUI::handleFrameDataEvents>(this);
    this->eventDispatcher.sink<KeyboardEvent>()
                         .connect<&DebugUI::handleKeyboardEvents>(this);
    this->eventDispatcher.sink<MouseWheelEvent>()
                         .connect<&DebugUI::handleMouseWheelEvents>(this);
    this->eventDispatcher.sink<PPMRatioChange>()
                         .connect<&DebugUI::handlePPMRatioChangeEvents>(this);
    this->eventDispatcher.sink<GameTimeWarpEvent>()
                         .connect<&DebugUI::handleGameTimeWarpEvents>(this);
  }

  void DebugUI::update()
  {
    this->dispatchTimeWarpEvents();
    this->dispatchGameTimerStatusEvents();
  }

  void DebugUI::render()
  {
    if (this->isDebugUIDisplayed) {
      this->buildMenu();
    
      if (this->isCameraControlsDisplayed) {
        this->buildCameraControls();
      }

      if (this->isPerformanceMetricsDisplayed) {
        this->buildPerformanceMetrics();
      }

      if (this->isTimeControlsDisplayed) {
        this->buildTimeControls();
      }
    }
  }

  void DebugUI::buildMenu()
  {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("Windows")) {
        eastl::string camControlsText("Show Camera Controls");
        if (this->isCameraControlsDisplayed) {
          camControlsText.insert(0, "/ ");
        }

        eastl::string performanceMetricsText("Show Performance Metrics");
        if (this->isPerformanceMetricsDisplayed) {
          performanceMetricsText.insert(0, "/ ");
        }

        eastl::string timeControlsText("Show Time Controls");
        if (this->isTimeControlsDisplayed) {
          timeControlsText.insert(0, "/ ");
        }

        if (ImGui::MenuItem(camControlsText.c_str())) {
          this->isCameraControlsDisplayed = !this->isCameraControlsDisplayed;
        }

        if (ImGui::MenuItem(performanceMetricsText.c_str())) {
          this->isPerformanceMetricsDisplayed =
            !this->isPerformanceMetricsDisplayed;
        }

        if (ImGui::MenuItem(timeControlsText.c_str())) {
          this->isTimeControlsDisplayed = !this->isTimeControlsDisplayed;
        }

        ImGui::EndMenu();
      }

      ImGui::EndMainMenuBar();
    }
  }

  void DebugUI::buildPerformanceMetrics()
  {
    ImGui::Begin("Performance Metrics");
    ImGui::Text("FPS: %d", this->metrics.fps);
    ImGui::Text("Time Delta: %f", this->metrics.timeDelta);
    ImGui::Text("App Time Delta: %f", this->metrics.appTimeDelta);
    ImGui::Text("No. of Frames: %d", this->metrics.numFrames);
    ImGui::End();
  }

  void DebugUI::buildCameraControls()
  {
    ImGui::Begin("Camera Controls");
    
    ImGui::Checkbox("Enable Free-Fly", &this->isFreeFlyEnabled);

    if (this->isFreeFlyEnabled) {
      ImGui::InputFloat("X", &this->camera.getGPUCamera()->x);
      ImGui::InputFloat("Y", &this->camera.getGPUCamera()->y);
      ImGui::InputFloat("Z", &this->camera.getGPUCamera()->z);
      ImGui::InputFloat("Z Near", &this->camera.getGPUCamera()->z_near);
      ImGui::InputFloat("Z Far", &this->camera.getGPUCamera()->z_far);
      ImGui::InputFloat("Zoom X", &this->camera.getGPUCamera()->zoom_x);
      ImGui::InputFloat("Zoom Y", &this->camera.getGPUCamera()->zoom_y);
      ImGui::InputFloat("Zoom X Delta", &this->cameraZoomXDelta);
      ImGui::InputFloat("Zoom Y Delta", &this->cameraZoomYDelta);
      ImGui::SliderFloat("Angle", &this->cameraAngle, 0.0f, 360.0f);
      ImGui::InputFloat("Movement Speed", &this->cameraMovementSpeed);
    }
    
    ImGui::End();

    this->camera.setZoomXDelta(this->cameraZoomXDelta);
    this->camera.setZoomYDelta(this->cameraZoomYDelta);
    this->camera.setAngle(this->cameraAngle);
  }

  void DebugUI::buildTimeControls()
  {
    ImGui::Begin("Time Controls");

    ImGui::Text("Play/Pause Game Time");
    if (ImGui::Button(this->isGamePlaying ? "Pause" : "Play")) {
      this->isGamePlaying = !this->isGamePlaying;
    }

    ImGui::Text("Slow Down/Accelerate Game Time");
    ImGui::SliderFloat("Time Warp Factor",
                       &this->gameTimeWarpFactor,
                       0.0f,
                       16.0f);
    ImGui::End();
  }

  void buildMouseDebugWindow()
  {
    ImGui::Begin("Mouse Debug Window");

    ImGui::Text("Mouse X: ");
    ImGui::Text("Mouse Y: ");

    ImGui::End();
  }

  void DebugUI::dispatchTimeWarpEvents()
  {
    this->eventDispatcher.enqueue<GameTimeWarpEvent>(this->gameTimeWarpFactor);
  }

  void DebugUI::dispatchGameTimerStatusEvents()
  {
    this->eventDispatcher.enqueue<GameTimerStatusEvent>(this->isGamePlaying);
  }

  void DebugUI::handleFrameDataEvents(const FrameDataEvent& e)
  {
    this->metrics = PerformanceMetrics{
      e.fps,
      e.numFrames,
      e.timeDelta,
      e.appTimeDelta
    };
  }

  void DebugUI::handleKeyboardEvents(const KeyboardEvent& e)
  {
    if (e.keyState == KeyState::KEY_DOWN && e.numRepeats == 0) {
      switch (e.keyCode) {
        case SDLK_F5:
          this->isDebugUIDisplayed = !this->isDebugUIDisplayed;
          this->isFreeFlyEnabled = false;
          this->gameTimeWarpFactor = 1.0f;
          break;
      }
    }

    if (this->isFreeFlyEnabled) {
      if (e.keyState == KeyState::KEY_DOWN) {
        switch (e.keyCode) {
          case SDLK_w:
            this->camera.moveY(
              -metersToPixels(this->cameraMovementSpeed, this->ppmRatio)
              * this->metrics.appTimeDelta
            );          
            break;
          case SDLK_a:
            this->camera.moveX(
              -metersToPixels(this->cameraMovementSpeed, this->ppmRatio)
              * this->metrics.appTimeDelta
            );
            break;
          case SDLK_s:
            this->camera.moveY(
              metersToPixels(this->cameraMovementSpeed, this->ppmRatio)
              * this->metrics.appTimeDelta
            );
            break;
          case SDLK_d:
            this->camera.moveX(
              metersToPixels(this->cameraMovementSpeed, this->ppmRatio)
              * this->metrics.appTimeDelta
            );
            break;
        }
      }
    }
  }

  void DebugUI::handleMouseWheelEvents(const MouseWheelEvent& e)
  {
    if (this->isFreeFlyEnabled) {
      if (e.event.wheel.y > 0) {
        // Scroll up.
        this->camera.zoom(CameraZoomState::IN);
      } else if (e.event.wheel.y < 0) {
        // Scroll down.
        this->camera.zoom(CameraZoomState::OUT);
      }
    }
  }

  void DebugUI::handlePPMRatioChangeEvents(const PPMRatioChange& e)
  {
    this->ppmRatio = e.newValue;
  }

  void DebugUI::handleGameTimeWarpEvents(const GameTimeWarpEvent& e)
  {
    this->gameTimeWarpFactor = e.timeWarpFactor;
  }
}
