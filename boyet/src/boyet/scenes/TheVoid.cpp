#include <cmath>
#include <iostream>

#include <EASTL/array.h>
#include <EASTL/vector.h>
#include <entt/entt.hpp>
#include <SDL2/SDL.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/asset_functions.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/components/Position.hpp>
#include <corex/core/components/Renderable.hpp>
#include <corex/core/components/RenderPolygon.hpp>
#include <corex/core/components/RenderRectangle.hpp>
#include <corex/core/components/RenderableType.hpp>
#include <corex/core/components/Sprite.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/ds/Vec2.hpp>
#include <corex/core/events/sys_events.hpp>

#include <boyet/scenes/TheVoid.hpp>

namespace boyet::scenes
{
  TheVoid::TheVoid(entt::registry& registry,
                   entt::dispatcher& eventDispatcher,
                   corex::core::AssetManager& assetManager)
    : corex::core::Scene(registry, eventDispatcher, assetManager)
    , numIntersections(0) {}

  void TheVoid::init()
  {
    std::cout << "The Void is being initialized..." << std::endl;

    this->eventDispatcher.sink<corex::core::WindowEvent>()
                         .connect<&TheVoid::handleWindowEvents>(this);

    auto texture = this->assetManager.getTexture("default-texture");

    constexpr int numTestEntities = 3;
    eastl::array<int, numTestEntities> divPositions = { 4, 2, 1 };
    for (int i = 0; i < numTestEntities; i++) {
      entt::entity e = this->registry.create();
      this->registry.emplace<corex::core::Position>(
        e,
        (640.f / divPositions[i]) - (96.f / 2.f),
        (480.f / divPositions[i]) - (96.f / 2.f),
        96.f,
        static_cast<int8_t>(0));
      this->registry.emplace<corex::core::Sprite>(e, 0, 0, 96, 96, texture);
      this->registry.emplace<corex::core::Renderable>(
        e,
        corex::core::RenderableType::SPRITE);
    }

    entt::entity e = this->registry.create();
    this->registry.emplace<corex::core::Position>(e,
                                                  320.f,
                                                  320.f,
                                                  32.f,
                                                  static_cast<int8_t>(0));
    this->registry.emplace<corex::core::Renderable>(
      e,
      corex::core::RenderableType::SPRITE);
    corex::core::addSpritesheetToEntity(e,
                                        "sitting-dog",
                                        this->registry,
                                        this->assetManager);

    entt::entity rect0Entity = this->registry.create();
    this->registry.emplace<corex::core::Position>(rect0Entity,
                                                  240.f,
                                                  240.f,
                                                  0.f,
                                                  static_cast<int8_t>(0));
    this->registry.emplace<corex::core::Renderable>(
      rect0Entity,
      corex::core::RenderableType::PRIMITIVE_RECTANGLE);
    this->registry.emplace<corex::core::RenderRectangle>(
      rect0Entity,
      240.f,
      240.f,
      50.f,
      100.f,
      0.f,
      SDL_Color{0, 153, 255, 200},
      true);
    this->rectangle0Entity = rect0Entity;

    entt::entity rect1Entity = this->registry.create();
    this->registry.emplace<corex::core::Position>(rect1Entity,
                                                  240.f,
                                                  320.f,
                                                  0.f,
                                                  static_cast<int8_t>(0));
    this->registry.emplace<corex::core::Renderable>(
      rect1Entity,
      corex::core::RenderableType::PRIMITIVE_RECTANGLE);
    this->registry.emplace<corex::core::RenderRectangle>(
      rect1Entity,
      240.f,
      320.f,
      50.f,
      100.f,
      90.f,
      SDL_Color{51, 204, 51, 200},
      true);
    this->rectangle1Entity = rect1Entity;
  }

  void TheVoid::update(float timeDelta)
  {
    this->registry.patch<corex::core::RenderRectangle>(
      this->rectangle0Entity,
      [&timeDelta](corex::core::RenderRectangle& renderRect) {
        renderRect.angle = fmod(renderRect.angle + (10.f * timeDelta), 360.f);
      });
    this->registry.patch<corex::core::RenderRectangle>(
      this->rectangle1Entity,
      [&timeDelta](corex::core::RenderRectangle& renderRect) {
        renderRect.angle = fmod(renderRect.angle + (10.f * timeDelta), 360.f);
      });

    auto& renderRect0 = this->registry
                             .get<corex::core::RenderRectangle>(
                                this->rectangle0Entity);
    auto& renderRect1 = this->registry
                             .get<corex::core::RenderRectangle>(
                                this->rectangle1Entity);
    auto rect0 = corex::core::Rectangle{
      renderRect0.x,
      renderRect0.y,
      renderRect0.width,
      renderRect0.height,
      renderRect0.angle
    };
    auto rect1 = corex::core::Rectangle{
      renderRect1.x,
      renderRect1.y,
      renderRect1.width,
      renderRect1.height,
      renderRect1.angle
    };

    if (areTwoRectsIntersecting(rect0, rect1)) {
      if (!this->registry.valid(this->intersectionEntity)) {
        this->intersectionEntity = this->registry.create();
        this->registry.emplace<corex::core::Position>(this->intersectionEntity,
                                                      0.f,
                                                      0.f,
                                                      32.f,
                                                      static_cast<int8_t>(1));
        this->registry.emplace<corex::core::Renderable>(
          this->intersectionEntity,
          corex::core::RenderableType::PRIMITIVE_POLYGON);
        this->registry.emplace<corex::core::RenderPolygon>(
          this->intersectionEntity,
          eastl::vector<corex::core::Point>{},
          SDL_Color{255, 77, 77, 255},
          true);
      }

      this->registry.patch<corex::core::RenderPolygon>(
        this->intersectionEntity,
        [&rect0, &rect1](corex::core::RenderPolygon& renderPoly) {
          auto clippedPoly = clippedPolygonFromTwoRects(rect0, rect1);
          renderPoly.vertices = clippedPoly.vertices;
        }
      );
    } else {
      if (this->registry.valid(this->intersectionEntity)) {
        this->registry.destroy(this->intersectionEntity);
      }
    }
  }

  void TheVoid::dispose()
  {
    std::cout << "\nDisposing The Void scene. :-(" << std::endl;
  }

  void TheVoid::handleWindowEvents(const corex::core::WindowEvent& e)
  {
    if (e.event.window.event == SDL_WINDOWEVENT_CLOSE) {
      this->setSceneStatus(corex::core::SceneStatus::DONE);
    }
  }
}
