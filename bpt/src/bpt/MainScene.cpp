#include <iostream>

#include <imgui.h>
#include <imgui_impls/imgui_impl_opengl3.h>
#include <imgui_impls/imgui_impl_sdl.h>
#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/components/Position.hpp>
#include <corex/core/components/Renderable.hpp>
#include <corex/core/components/RenderableType.hpp>
#include <corex/core/components/RenderLineSegments.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/MouseButtonState.hpp>
#include <corex/core/systems/MouseButtonType.hpp>

#include <bpt/Context.hpp>
#include <bpt/MainScene.hpp>

namespace bpt
{
  MainScene::MainScene(entt::registry& registry,
                       entt::dispatcher& eventDispatcher,
                       corex::core::AssetManager& assetManager)
    : currentContext(Context::NO_ACTION)
    , wipBoundingArea()
    , wipBoundingAreaEntity()
    , corex::core::Scene(registry, eventDispatcher, assetManager) {}

  void MainScene::init()
  {
    std::cout << "MainScene is being initialized..." << std::endl;

    this->eventDispatcher.sink<corex::core::WindowEvent>()
                         .connect<&MainScene::handleWindowEvents>(this);
    this->eventDispatcher.sink<corex::core::MouseButtonEvent>()
                         .connect<&MainScene::handleMouseButtonEvents>(this);
    this->eventDispatcher.sink<corex::core::MouseMovementEvent>()
                         .connect<&MainScene::handleMouseMovementEvents>(this);
  }

  void MainScene::update(float timeDelta)
  {
    this->buildConstructBoundingAreaWindow();

    switch (this->currentContext) {
      case Context::NO_ACTION: {
        if (this->registry.valid(this->wipBoundingAreaEntity)) {
          this->registry.destroy(this->wipBoundingAreaEntity);
        }
      } break;
      case Context::DRAW_AREA_BOUND: {
        if (!this->registry.valid(this->wipBoundingAreaEntity)) {
          this->wipBoundingArea.vertices.clear();
          this->wipBoundingAreaEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->wipBoundingAreaEntity,
            0.f,
            0.f,
            32.f,
            static_cast<int8_t>(1));
          this->registry.emplace<corex::core::Renderable>(
            this->wipBoundingAreaEntity,
            corex::core::RenderableType::LINE_SEGMENTS);
          this->registry.emplace<corex::core::RenderLineSegments>(
            this->wipBoundingAreaEntity,
            eastl::vector<corex::core::Point>{},
            SDL_Color{64, 64, 64, 255});
        } else {
          this->registry.patch<corex::core::RenderLineSegments>(
            this->wipBoundingAreaEntity,
            [this](corex::core::RenderLineSegments& segments) {
              segments.vertices = this->wipBoundingArea.vertices;
            }
          );
        }
      } break;
    }
  }

  void MainScene::dispose()
  {
    std::cout << "Disposing MainScene. Bleep, bloop, zzzz." << std::endl;
  }

  void MainScene::buildConstructBoundingAreaWindow()
  {
    ImGui::Begin("Bounding Area");

    switch (this->currentContext) {
      case Context::NO_ACTION:
        if (ImGui::Button("Create Bounding Area")) {
          this->currentContext = Context::DRAW_AREA_BOUND;

          // Just doing this to make sure we don't get unneeded vertices.
          this->wipBoundingArea.vertices.clear();
          this->wipBoundingArea.vertices.push_back(corex::core::Point{});
        }
        break;
      case Context::DRAW_AREA_BOUND:
        ImGui::Text("Press the Right Mouse Button to Stop.");
    }

    ImGui::Text("Bound Area Vertices");
    ImGui::BeginChild("Vertices List");
    for (corex::core::Point& pt : this->wipBoundingArea.vertices) {
      ImGui::Text("%f, %f", pt.x, pt.y);
    }
    ImGui::EndChild();

    ImGui::End();
  }

  void MainScene::handleWindowEvents(const corex::core::WindowEvent& e)
  {
    if (e.event.window.event == SDL_WINDOWEVENT_CLOSE) {
      this->setSceneStatus(corex::core::SceneStatus::DONE);
    }
  }

  void
  MainScene::handleMouseButtonEvents(const corex::core::MouseButtonEvent& e)
  {
    switch (this->currentContext) {
      case Context::NO_ACTION:
        break;
      case Context::DRAW_AREA_BOUND:
        if (e.buttonType == corex::core::MouseButtonType::MOUSE_BUTTON_LEFT
            && e.buttonState == corex::core
                                     ::MouseButtonState::MOUSE_BUTTON_DOWN
            && e.numRepeats == 0) {
          this->wipBoundingArea.vertices.push_back(corex::core::Point{
            static_cast<float>(e.x),
            static_cast<float>(e.y)
          });

          if (this->wipBoundingArea.vertices.size() == 1) {
            this->wipBoundingArea.vertices.push_back(corex::core::Point{
              static_cast<float>(e.x),
              static_cast<float>(e.y)
            });
          }
        } else if (
            e.buttonType == corex::core::MouseButtonType::MOUSE_BUTTON_RIGHT
            && e.buttonState == corex::core
                                     ::MouseButtonState::MOUSE_BUTTON_DOWN
            && e.numRepeats == 0) {
          this->currentContext = Context::NO_ACTION;
        }
        break;
    }
  }

  void
  MainScene::handleMouseMovementEvents(const corex::core::MouseMovementEvent& e)
  {
    switch (this->currentContext) {
      case Context::NO_ACTION:
        break;
      case Context::DRAW_AREA_BOUND:
        if (!this->wipBoundingArea.vertices.empty()) {
          int32_t lastElementIndex = this->wipBoundingArea.vertices.size() - 1;
          this->wipBoundingArea.vertices[lastElementIndex] = corex::core::Point{
            static_cast<float>(e.x),
            static_cast<float>(e.y)
          };
        }
        break;
    }
  }
}
