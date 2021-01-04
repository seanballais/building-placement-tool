#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <EASTL/vector.h>
#include <entt/entt.hpp>
#include <imgui.h>
#include <implot/implot.h>
#include <nlohmann/json.hpp>
#include <SDL2/SDL.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/CameraZoomState.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/components/Position.hpp>
#include <corex/core/components/Renderable.hpp>
#include <corex/core/components/RenderableType.hpp>
#include <corex/core/components/RenderCircle.hpp>
#include <corex/core/components/RenderLineSegments.hpp>
#include <corex/core/components/RenderRectangle.hpp>
#include <corex/core/components/RenderPolygon.hpp>
#include <corex/core/components/Text.hpp>
#include <corex/core/ds/Circle.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/MouseScrollEvent.hpp>
#include <corex/core/events/sys_events.hpp>
#include <corex/core/systems/KeyState.hpp>
#include <corex/core/systems/MouseButtonState.hpp>
#include <corex/core/systems/MouseButtonType.hpp>

#include <bpt/Context.hpp>
#include <bpt/json_specializations.hpp>
#include <bpt/MainScene.hpp>
#include <bpt/SelectionType.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/InputBuilding.hpp>

namespace bpt
{
  MainScene::MainScene(entt::registry& registry,
                       entt::dispatcher& eventDispatcher,
                       corex::core::AssetManager& assetManager,
                       corex::core::Camera& camera)
    : currentContext(Context::NO_ACTION)
    , geneticAlgo()
    , gaSettings({
        0.25f,
        25,
        1000,
        4,
        5,
        5.f,
        5.f,
        1.0f,
        true,
        SelectionType::NONE
      })
    , currentSolution(nullptr)
    , solutions()
    , isGAThreadRunning(false)
    , hasSolutionBeenSetup(false)
    , doesInputDataExist(false)
    , doesInputBoundingAreaFieldExist(false)
    , doesInputBuildingsExist(false)
    , doesGASettingsFieldExist(false)
    , doFloodProneAreasExist(false)
    , doLandslideProneAreasExist(false)
    , isCloseAreaTriggerEnabled(false)
    , showGAResultsAverage(true)
    , showGAResultsBest(true)
    , showGAResultsWorst(true)
    , needUpdateBuildingRenderMode(false)
    , isGATimelinePlaying(false)
    , closeAreaTriggerCircle(corex::core::Circle{
        corex::core::Point{ 0.f, 0.f }, 0.0f
      })
    , cameraMovementSpeed(15.f)
    , timeDelta(0.f)
    , currSelectedGen(0)
    , currSelectedGenSolution(0)
    , gaTimelinePlaybackSpeed(1)
    , wipBoundingArea()
    , wipHazardArea()
    , boundingArea()
    , boundingAreaTriangles()
    , floodProneAreas()
    , landslideProneAreas()
    , wipBoundingAreaEntity(entt::null)
    , wipHazardAreaEntity(entt::null)
    , boundingAreaEntity(entt::null)
    , closeAreaTriggerEntity(entt::null)
    , boundingAreaTriangleEntities()
    , floodProneAreaEntities()
    , landslideProneAreaEntities()
    , inputBuildings()
    , buildingEntities()
    , flowRates()
    , buildingTextEntities()
    , floodProneAreaTextEntities()
    , landslideProneAreaTextEntities()
    , recentGARunAvgFitnesses()
    , recentGARunBestFitnesses()
    , recentGARunWorstFitnesses()
    , inputData()
    , corex::core::Scene(registry, eventDispatcher, assetManager, camera) {}

  void MainScene::init()
  {
    std::cout << "MainScene is being initialized..." << std::endl;

    this->eventDispatcher.sink<corex::core::WindowEvent>()
                         .connect<&MainScene::handleWindowEvents>(this);
    this->eventDispatcher.sink<corex::core::KeyboardEvent>()
                         .connect<&MainScene::handleKeyboardEvents>(this);
    this->eventDispatcher.sink<corex::core::MouseButtonEvent>()
                         .connect<&MainScene::handleMouseButtonEvents>(this);
    this->eventDispatcher.sink<corex::core::MouseMovementEvent>()
                         .connect<&MainScene::handleMouseMovementEvents>(this);
    this->eventDispatcher.sink<corex::core::MouseScrollEvent>()
                         .connect<&MainScene::handleMouseScrollEvents>(this);

    // Parse the input file.
    std::filesystem::path inputFilePath = corex::core::getBinFolder()
                                          / "data/input_data.bptdat";
    if (std::filesystem::exists(inputFilePath)) {
      std::ifstream inputFile(inputFilePath);
      if (inputFile) {
        std::ostringstream fileStrStream;
        fileStrStream << inputFile.rdbuf();
        if (nlohmann::json::accept(fileStrStream.str())) {
          this->doesInputDataExist = true;
          this->inputData = nlohmann::json::parse(fileStrStream.str());

          if (this->inputData.contains("boundingAreaVertices")) {
            for (auto& vertex : this->inputData["boundingAreaVertices"]) {
              // Sigh. Should have used a JSON object here.
              this->boundingArea.vertices.push_back(corex::core::Point{
                vertex[0].get<float>(), vertex[1].get<float>()
              });
            }

            this->boundingAreaTriangles = cx::earClipTriangulate(
              this->boundingArea);

            this->doesInputBoundingAreaFieldExist = true;
          }

          if (this->inputData.contains("inputBuildings")) {
            int32_t numBuildings = this->inputData["inputBuildings"].size();
            int32_t buildingIndex = 0;
            for (auto& building : this->inputData["inputBuildings"]) {
              // Sigh. Should have used a JSON object here.
              this->inputBuildings.push_back(InputBuilding{
                building[0].get<float>(), building[1].get<float>()
              });

              eastl::vector<float> buildingFlowRates;
              if (building[2].size() == 0) {
                buildingFlowRates = eastl::vector<float>(numBuildings, 1.f);
                buildingFlowRates[buildingIndex] = 0.f;
              } else {
                for (auto flowRate : building[2]) {
                  buildingFlowRates.push_back(flowRate.get<float>());
                }
              }

              this->flowRates.push_back(buildingFlowRates);

              buildingIndex++;
            }
            this->doesInputBuildingsExist = true;

            assert(this->inputBuildings.size() == this->flowRates.size());
          }

          if (this->inputData.contains("floodProneAreas")) {
            for (auto& area : this->inputData["floodProneAreas"]) {
              this->floodProneAreas.push_back();
              this->floodProneAreaEntities.push_back(entt::null);
              this->floodProneAreaTextEntities.push_back(entt::null);
              for (auto& vertex : area) {
                this->floodProneAreas.back().vertices.push_back(
                  corex::core::Point{
                    vertex[0].get<float>(),
                    vertex[1].get<float>()
                  }
                );
              }
            }

            this->doFloodProneAreasExist = true;
          }

          if (this->inputData.contains("landslideProneAreas")) {
            for (auto& area : this->inputData["landslideProneAreas"]) {
              this->landslideProneAreas.push_back();
              this->landslideProneAreaEntities.push_back(entt::null);
              this->landslideProneAreaTextEntities.push_back(entt::null);
              for (auto& vertex : area) {
                this->landslideProneAreas.back().vertices.push_back(
                  corex::core::Point{
                    vertex[0].get<float>(),
                    vertex[1].get<float>()
                  }
                );
              }
            }

            this->doLandslideProneAreasExist = true;
          }

          if (this->inputData.contains("gaSettings")) {
            nlohmann::json gaSettingsJSON = this->inputData["gaSettings"];
            this->gaSettings.mutationRate = gaSettingsJSON["mutationRate"]
                                              .get<float>();
            this->gaSettings.populationSize = gaSettingsJSON["populationSize"]
                                                .get<int32_t>();
            this->gaSettings.numGenerations = gaSettingsJSON["numGenerations"]
                                                .get<int32_t>();
            this->gaSettings.tournamentSize = gaSettingsJSON["tournamentSize"]
                                                .get<int32_t>();
            this->gaSettings.numPrevGenOffsprings =
              gaSettingsJSON["numPrevGenOffsprings"].get<int32_t>();
            this->gaSettings.floodProneAreaPenalty =
              gaSettingsJSON["floodProneAreaPenalty"].get<float>();
            this->gaSettings.landslideProneAreaPenalty =
              gaSettingsJSON["landslideProneAreaPenalty"].get<float>();
            this->gaSettings.buildingDistanceWeight =
              gaSettingsJSON["buildingDistanceWeight"].get<float>();
            this->gaSettings.isLocalSearchEnabled =
              gaSettingsJSON["isLocalSearchEnabled"].get<bool>();
            this->gaSettings.selectionType =
              gaSettingsJSON["selectionType"].get<SelectionType>();

            this->doesGASettingsFieldExist = true;
          }
        }
      }
    }
  }

  void MainScene::update(float timeDelta)
  {
    this->timeDelta = timeDelta;

    this->handleGATimelinePlayback(timeDelta);

    if (this->solutions.size() > 0) {
      this->currentSolution = &(this->solutions[this->currSelectedGen]
                                               [this->currSelectedGenSolution]);
    }

    if (this->currentSolution
        && !this->hasSolutionBeenSetup
        && !this->isGAThreadRunning) {
      this->clearCurrentlyRenderedSolution();

      for (int32_t i = 0; i < this->currentSolution->getNumBuildings(); i++) {
        entt::entity e = this->registry.create();
        this->registry.emplace<corex::core::Position>(
          e,
          this->currentSolution->getBuildingXPos(i),
          this->currentSolution->getBuildingYPos(i),
          0.f,
          static_cast<int8_t>(1));
        this->registry.emplace<corex::core::Renderable>(
          e,
          corex::core::RenderableType::PRIMITIVE_RECTANGLE);
        this->registry.emplace<corex::core::RenderRectangle>(
          e,
          this->currentSolution->getBuildingXPos(i),
          this->currentSolution->getBuildingYPos(i),
          this->inputBuildings[i].width,
          this->inputBuildings[i].length,
          this->currentSolution->getBuildingRotation(i),
          SDL_Color{0, 102, 51, 255},
          true);

        this->buildingEntities.push_back(e);
      }

      for (int32_t i = 0; i < this->currentSolution->getNumBuildings(); i++) {
        // We are using two separate loops to update the building and building
        // text entities for performance reasons. Doing this will ensure that
        // there will be more cache hits than cache misses.
        entt::entity e = this->registry.create();
        this->registry.emplace<corex::core::Position>(
          e,
          this->currentSolution->getBuildingXPos(i),
          this->currentSolution->getBuildingYPos(i),
          0.f,
          static_cast<int8_t>(5));
        this->registry.emplace<corex::core::Renderable>(
          e,
          corex::core::RenderableType::TEXT);
        this->registry.emplace<corex::core::Text>(
          e,
          eastl::to_string(i),
          this->assetManager.getFont("liberation-sans-regular-font", 15),
          SDL_Color{ 255, 255, 255, 255 });

        this->buildingTextEntities.push_back(e);
      }

      this->currentSolution->setFitness(
        this->geneticAlgo.getSolutionFitness(
          *(this->currentSolution),
          this->inputBuildings,
          this->flowRates,
          this->floodProneAreas,
          this->landslideProneAreas,
          this->gaSettings.floodProneAreaPenalty,
          this->gaSettings.landslideProneAreaPenalty,
          this->gaSettings.buildingDistanceWeight));

      this->hasSolutionBeenSetup = true;
    }

    if (this->needUpdateBuildingRenderMode) {
      // Switch rendering mode for the buildings from wireframe mode and filled
      // mode.
      for (entt::entity e : this->buildingEntities) {
        this->registry.patch<corex::core::RenderRectangle>(
          e,
          [this](corex::core::RenderRectangle& rect) {
            rect.isFilled = !rect.isFilled;
          }
        );

        this->needUpdateBuildingRenderMode = false;
      }
    }

    switch (this->currentContext) {
      case Context::NO_ACTION: {
        if (!this->registry.valid(this->boundingAreaEntity)) {
          this->boundingAreaEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->boundingAreaEntity,
            0.f,
            0.f,
            3.f,
            static_cast<int8_t>(2));
          this->registry.emplace<corex::core::Renderable>(
            this->boundingAreaEntity,
            corex::core::RenderableType::PRIMITIVE_POLYGON);
          this->registry.emplace<corex::core::RenderPolygon>(
            this->boundingAreaEntity,
            eastl::vector<corex::core::Point>{},
            SDL_Color{ 64, 64, 64, 255},
            false);

          for (auto& triangle : this->boundingAreaTriangles) {
            entt::entity triangleEntity = this->registry.create();
            this->registry.emplace<cx::Position>(triangleEntity,
                                                 0.f,
                                                 0.f,
                                                 2.f,
                                                 static_cast<int8_t>(2));
            this->registry.emplace<cx::Renderable>(
              triangleEntity,
              cx::RenderableType::PRIMITIVE_POLYGON);
            this->registry.emplace<cx::RenderPolygon>(
              triangleEntity,
              eastl::vector<corex::core::Point>{},
              SDL_Color{ 255, 0, 0, 255 },
              false);

            this->boundingAreaTriangleEntities.push_back(triangleEntity);
          }
        } else {
          this->registry.patch<corex::core::RenderPolygon>(
            this->boundingAreaEntity,
            [this](corex::core::RenderPolygon& poly) {
              poly.vertices = this->boundingArea.vertices;
            }
          );

          for (int32_t i = 0; i < this->boundingAreaTriangles.size(); i++) {
            eastl::vector<cx::Point> triangleVertices;
            for (auto& v : this->boundingAreaTriangles[i].vertices) {
              triangleVertices.push_back(v);
            }

            this->registry.patch<corex::core::RenderPolygon>(
              this->boundingAreaTriangleEntities[i],
              [&triangleVertices](corex::core::RenderPolygon& poly) {
                poly.vertices = triangleVertices;
              }
            );
          }
        }

        // Draw hazard areas.
        // Draw flood-prone areas.
        for (int32_t i = 0; i < this->floodProneAreas.size(); i++) {
          if (!this->registry.valid(this->floodProneAreaEntities[i])) {
            this->floodProneAreaEntities[i] = this->registry.create();
            this->registry.emplace<corex::core::Position>(
              this->floodProneAreaEntities[i],
              0.f,
              0.f,
              1.f,
              static_cast<int8_t>(0));
            this->registry.emplace<corex::core::Renderable>(
              this->floodProneAreaEntities[i],
              corex::core::RenderableType::PRIMITIVE_POLYGON);
            this->registry.emplace<corex::core::RenderPolygon>(
              this->floodProneAreaEntities[i],
              eastl::vector<corex::core::Point>{},
              SDL_Color{ 0, 115, 153, 255 },
              true);
          } else {
            this->registry.patch<corex::core::RenderPolygon>(
              this->floodProneAreaEntities[i],
              [this, &i](corex::core::RenderPolygon& poly) {
                poly.vertices = this->floodProneAreas[i].vertices;
              }
            );
          }
        }

        for (int32_t i = 0; i < this->floodProneAreas.size(); i++) {
          if (!this->registry.valid(this->floodProneAreaTextEntities[i])) {
            // Add the flood-prone area number.
            this->floodProneAreaTextEntities[i] = this->registry.create();
            auto polygonCentroid = corex::core::getPolygonCentroid(
              this->floodProneAreas[i]);
            this->registry.emplace<corex::core::Position>(
              this->floodProneAreaTextEntities[i],
              polygonCentroid.x,
              polygonCentroid.y,
              0.f,
              static_cast<int8_t>(10));
            this->registry.emplace<corex::core::Renderable>(
              this->floodProneAreaTextEntities[i],
              corex::core::RenderableType::TEXT);
            this->registry.emplace<corex::core::Text>(
              this->floodProneAreaTextEntities[i],
              eastl::to_string(i),
              this->assetManager.getFont("liberation-sans-regular-font", 15),
              SDL_Color{ 255, 255, 255, 255 });
          }
        }

        // Draw landslide-prone areas.
        for (int32_t i = 0; i < this->landslideProneAreas.size(); i++) {
          if (!this->registry.valid(this->landslideProneAreaEntities[i])) {
            this->landslideProneAreaEntities[i] = this->registry.create();
            this->registry.emplace<corex::core::Position>(
              this->landslideProneAreaEntities[i],
              0.f,
              0.f,
              1.f,
              static_cast<int8_t>(0));
            this->registry.emplace<corex::core::Renderable>(
              this->landslideProneAreaEntities[i],
              corex::core::RenderableType::PRIMITIVE_POLYGON);
            this->registry.emplace<corex::core::RenderPolygon>(
              this->landslideProneAreaEntities[i],
              eastl::vector<corex::core::Point>{},
              SDL_Color{ 102, 34, 0, 255 },
              true);
          } else {
            this->registry.patch<corex::core::RenderPolygon>(
              this->landslideProneAreaEntities[i],
              [this, &i](corex::core::RenderPolygon& poly) {
                poly.vertices = this->landslideProneAreas[i].vertices;
              }
            );
          }
        }

        for (int32_t i = 0; i < this->landslideProneAreas.size(); i++) {
          if (!this->registry.valid(this->landslideProneAreaTextEntities[i])) {
            // Add the landslide-prone area number.
            this->landslideProneAreaTextEntities[i] = this->registry.create();
            auto polygonCentroid = corex::core::getPolygonCentroid(
              this->landslideProneAreas[i]);
            this->registry.emplace<corex::core::Position>(
              this->landslideProneAreaTextEntities[i],
              polygonCentroid.x,
              polygonCentroid.y,
              0.f,
              static_cast<int8_t>(10));
            this->registry.emplace<corex::core::Renderable>(
              this->landslideProneAreaTextEntities[i],
              corex::core::RenderableType::TEXT);
            this->registry.emplace<corex::core::Text>(
              this->landslideProneAreaTextEntities[i],
              eastl::to_string(i),
              this->assetManager.getFont("liberation-sans-regular-font", 15),
              SDL_Color{ 255, 255, 255, 255 });
          }
        }

        if (this->registry.valid(this->wipBoundingAreaEntity)) {
          this->registry.destroy(this->wipBoundingAreaEntity);
        }

        if (this->registry.valid(this->wipHazardAreaEntity)) {
          this->registry.destroy(this->wipHazardAreaEntity);
        }

        if (this->registry.valid(this->closeAreaTriggerEntity)) {
          this->registry.destroy(this->closeAreaTriggerEntity);
        }
      } break;
      case Context::DRAW_AREA_BOUND: {
        if (!this->registry.valid(this->wipBoundingAreaEntity)) {
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

        if (!this->registry.valid(this->closeAreaTriggerEntity)) {
          this->closeAreaTriggerEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->closeAreaTriggerEntity,
            0.f,
            0.f,
            32.f,
            static_cast<int8_t>(1));
          this->registry.emplace<corex::core::Renderable>(
            this->closeAreaTriggerEntity,
            corex::core::RenderableType::PRIMITIVE_CIRCLE);
          this->registry.emplace<corex::core::RenderCircle>(
            this->closeAreaTriggerEntity,
            this->closeAreaTriggerCircle.radius,
            SDL_Color{64, 64, 64, 255},
            true);
        } else {
          this->registry.patch<corex::core::Position>(
            this->closeAreaTriggerEntity,
            [this](corex::core::Position& pos) {
              pos.x = this->closeAreaTriggerCircle.position.x;
              pos.y = this->closeAreaTriggerCircle.position.y;
            }
          );
          this->registry.patch<corex::core::RenderCircle>(
            this->closeAreaTriggerEntity,
            [this](corex::core::RenderCircle& circle) {
              circle.radius = this->closeAreaTriggerCircle.radius;
            }
          );
        }

        if (this->registry.valid(this->boundingAreaEntity)) {
          this->boundingArea.vertices.clear();
          this->boundingAreaTriangles.clear();

          for (auto& e : this->boundingAreaTriangleEntities) {
            this->registry.destroy(e);
          }

          this->boundingAreaTriangleEntities.clear();

          this->registry.destroy(this->boundingAreaEntity);
        }
      } break;
      case Context::DRAW_FLOOD_PRONE_AREA:
      case Context::DRAW_LANDSLIDE_PRONE_AREA: {
        SDL_Color wipHazardAreaColour;
        switch (this->currentContext) {
          case Context::DRAW_FLOOD_PRONE_AREA:
            wipHazardAreaColour = SDL_Color{ 0, 115, 153, 255 };
            break;
          case Context::DRAW_LANDSLIDE_PRONE_AREA:
            wipHazardAreaColour = SDL_Color{ 102, 32, 0, 255};
            break;
          default:
            break;
        }

        if (!this->registry.valid(this->wipHazardAreaEntity)) {
          this->wipHazardAreaEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->wipHazardAreaEntity,
            0.f,
            0.f,
            32.f,
            static_cast<int8_t>(-1));
          this->registry.emplace<corex::core::Renderable>(
            this->wipHazardAreaEntity,
            corex::core::RenderableType::LINE_SEGMENTS);
          this->registry.emplace<corex::core::RenderLineSegments>(
            this->wipHazardAreaEntity,
            eastl::vector<corex::core::Point>{},
            wipHazardAreaColour);
        } else {
          this->registry.patch<corex::core::RenderLineSegments>(
            this->wipHazardAreaEntity,
            [this](corex::core::RenderLineSegments& segments) {
              segments.vertices = this->wipHazardArea.vertices;
            }
          );
        }

        if (!this->registry.valid(this->closeAreaTriggerEntity)) {
          this->closeAreaTriggerEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->closeAreaTriggerEntity,
            0.f,
            0.f,
            32.f,
            static_cast<int8_t>(-1));
          this->registry.emplace<corex::core::Renderable>(
            this->closeAreaTriggerEntity,
            corex::core::RenderableType::PRIMITIVE_CIRCLE);
          this->registry.emplace<corex::core::RenderCircle>(
            this->closeAreaTriggerEntity,
            this->closeAreaTriggerCircle.radius,
            wipHazardAreaColour,
            true);
        } else {
          this->registry.patch<corex::core::Position>(
            this->closeAreaTriggerEntity,
            [this](corex::core::Position& pos) {
              pos.x = this->closeAreaTriggerCircle.position.x;
              pos.y = this->closeAreaTriggerCircle.position.y;
            }
          );
          this->registry.patch<corex::core::RenderCircle>(
            this->closeAreaTriggerEntity,
            [this](corex::core::RenderCircle& circle) {
              circle.radius = this->closeAreaTriggerCircle.radius;
            }
          );
        }
      } break;
      default:
        break;
    }

    this->buildConstructBoundingAreaWindow();
    this->buildWarningWindow();
    this->buildInputBuildingsWindow();
    this->buildHazardsWindow();
    this->buildFlowRateWindow();
    this->buildGAControlsWindow();
    this->buildGAResultsWindow();
    this->buildDebugSolutionWindow();
  }

  void MainScene::dispose()
  {
    std::cout << "Disposing MainScene. Bleep, bloop, zzzz." << std::endl;

    if (this->doesInputDataExist) {
      // Store the bounding area in the input data file.
      this->inputData["boundingAreaVertices"] = nlohmann::json::array();
      for (corex::core::Point& vertex : this->boundingArea.vertices) {
        this->inputData["boundingAreaVertices"].push_back(
          nlohmann::json::array({ vertex.x, vertex.y })
        );
      }

      this->inputData["inputBuildings"] = nlohmann::json::array();
      int32_t buildingIndex = 0;
      for (InputBuilding& building : this->inputBuildings) {
        nlohmann::json flowRates = nlohmann::json::array();
        for (float& flowRate : this->flowRates[buildingIndex]) {
          flowRates.push_back(flowRate);
        }

        this->inputData["inputBuildings"].push_back(
          nlohmann::json::array({ building.length, building.width, flowRates })
        );

        buildingIndex++;
      }

      this->inputData["gaSettings"] = nlohmann::json::object();
      this->inputData["gaSettings"]["mutationRate"] = this->gaSettings
                                                           .mutationRate;
      this->inputData["gaSettings"]["populationSize"] = this->gaSettings
                                                             .populationSize;
      this->inputData["gaSettings"]["numGenerations"] = this->gaSettings
                                                             .numGenerations;
      this->inputData["gaSettings"]["tournamentSize"] = this->gaSettings
                                                             .tournamentSize;
      this->inputData["gaSettings"]["numPrevGenOffsprings"] =
        this->gaSettings
             .numPrevGenOffsprings;
      this->inputData["gaSettings"]["floodProneAreaPenalty"] =
        this->gaSettings.floodProneAreaPenalty;
      this->inputData["gaSettings"]["landslideProneAreaPenalty"] =
        this->gaSettings.landslideProneAreaPenalty;
      this->inputData["gaSettings"]["buildingDistanceWeight"] =
        this->gaSettings.buildingDistanceWeight;
      this->inputData["gaSettings"]["isLocalSearchEnabled"] =
        this->gaSettings.isLocalSearchEnabled;
      this->inputData["gaSettings"]["selectionType"] =
        this->gaSettings.selectionType;

      this->inputData["floodProneAreas"] = nlohmann::json::array();
      for (corex::core::NPolygon& area : this->floodProneAreas) {
        nlohmann::json areaVertices = nlohmann::json::array();
        for (corex::core::Point& pt : area.vertices) {
          areaVertices.push_back(nlohmann::json::array({ pt.x, pt.y }));
        }

        this->inputData["floodProneAreas"].push_back(areaVertices);
      }

      this->inputData["landslideProneAreas"] = nlohmann::json::array();
      for (corex::core::NPolygon& area : this->landslideProneAreas) {
        nlohmann::json areaVertices = nlohmann::json::array();
        for (corex::core::Point& pt : area.vertices) {
          areaVertices.push_back(nlohmann::json::array({ pt.x, pt.y }));
        }

        this->inputData["landslideProneAreas"].push_back(areaVertices);
      }

      std::filesystem::path inputFilePath = corex::core::getBinFolder()
                                            / "data/input_data.bptdat";
      std::ofstream dataFile(inputFilePath, std::ios::trunc);
      dataFile << this->inputData;
    }
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
        ImGui::Text("Press the Right Mouse Button to Cancel.");
        break;
      case Context::DRAW_FLOOD_PRONE_AREA:
        ImGui::Text("Finish drawing a flood-prone area first.");
        break;
      case Context::DRAW_LANDSLIDE_PRONE_AREA:
        ImGui::Text("Finish drawing a landslide-prone area first.");
        break;
      default:
        break;
    }

    ImGui::Text("Bound Area Vertices");
    ImGui::BeginChild("Vertices List");
    if (this->currentContext == Context::NO_ACTION) {
      for (corex::core::Point& pt : this->boundingArea.vertices) {
        ImGui::Text("%f, %f", pt.x, pt.y);
      }
    } else if (this->currentContext == Context::DRAW_AREA_BOUND) {
      for (corex::core::Point& pt : this->wipBoundingArea.vertices) {
        ImGui::Text("%f, %f", pt.x, pt.y);
      }
    }
    ImGui::EndChild();

    ImGui::End();
  }

  void MainScene::buildHazardsWindow()
  {
    static int32_t removedFloodProneAreaIndex;
    static int32_t removedLandslideProneAreaIndex;
    removedFloodProneAreaIndex = -1;
    removedLandslideProneAreaIndex = -1;

    ImGui::Begin("Hazards");

    switch (this->currentContext) {
      case Context::NO_ACTION:
        if (ImGui::Button("Add Flood-Prone Area")) {
          this->currentContext = Context::DRAW_FLOOD_PRONE_AREA;

          // Just doing this to make sure we don't get unneeded vertices.
          this->wipHazardArea.vertices.clear();
          this->wipHazardArea.vertices.push_back(corex::core::Point{});
        }

        ImGui::SameLine();

        if (ImGui::Button("Add Landslide-Prone Area")) {
          this->currentContext = Context::DRAW_LANDSLIDE_PRONE_AREA;

          // Just doing this to make sure we don't get unneeded vertices.
          this->wipHazardArea.vertices.clear();
          this->wipHazardArea.vertices.push_back(corex::core::Point{});
        }

        break;
      case Context::DRAW_FLOOD_PRONE_AREA:
        ImGui::Text("Press RMB to cancel drawing a flood-prone area.");
        break;
      case Context::DRAW_LANDSLIDE_PRONE_AREA:
        ImGui::Text("Press RMB to cancel drawing a landslide-prone area.");
        break;
      case Context::DRAW_AREA_BOUND:
        ImGui::Text("Finish drawing the area bound first.");
        break;
      default:
        break;
    }

    ImGui::Separator();

    ImGui::BeginChild("Hazard Areas List");
    if (ImGui::CollapsingHeader("Flood-Prone Areas")) {
      for (int32_t i = 0; i < this->floodProneAreas.size(); i++) {
        corex::core::NPolygon& area = this->floodProneAreas[i];
        char areaText[23];
        char removeAreaBtnText[21];

        sprintf(areaText, "Flood-Prone Area #%d", i);
        sprintf(removeAreaBtnText, "Remove Area #%d##0", i);
        if (ImGui::TreeNode(areaText)) {
          for (corex::core::Point& pt : area.vertices) {
            ImGui::Text("%f, %f", pt.x, pt.y);
          }

          ImGui::TreePop();
        }

        if (ImGui::Button(removeAreaBtnText)) {
          removedFloodProneAreaIndex = i;
        }
      }

      if (this->currentContext == Context::DRAW_FLOOD_PRONE_AREA) {
        ImGui::Separator();

        ImGui::Text("New Area Vertices");
        for (corex::core::Point& pt : this->wipHazardArea.vertices) {
          ImGui::Text("%f, %f", pt.x, pt.y);
        }
      }
    }

    if (ImGui::CollapsingHeader("Landslide-Prone Areas")) {
      for (int32_t i = 0; i < this->landslideProneAreas.size(); i++) {
        corex::core::NPolygon& area = this->landslideProneAreas[i];
        char areaText[27];
        char removeAreaBtnText[21];

        sprintf(areaText, "Landslide-Prone Area #%d", i);
        sprintf(removeAreaBtnText, "Remove Area #%d##1", i);
        if (ImGui::TreeNode(areaText)) {
          for (corex::core::Point& pt : area.vertices) {
            ImGui::Text("%f, %f", pt.x, pt.y);
          }

          ImGui::TreePop();
        }

        if (ImGui::Button(removeAreaBtnText)) {
          removedLandslideProneAreaIndex = i;
        }
      }

      if (this->currentContext == Context::DRAW_LANDSLIDE_PRONE_AREA) {
        ImGui::Separator();

        ImGui::Text("New Area Vertices");
        for (corex::core::Point& pt : this->wipHazardArea.vertices) {
          ImGui::Text("%f, %f", pt.x, pt.y);
        }
      }
    }
    ImGui::EndChild();

    ImGui::End();

    if (removedFloodProneAreaIndex != -1) {
      this->registry.destroy(
        this->floodProneAreaEntities[removedFloodProneAreaIndex]);
      this->registry.destroy(
        this->floodProneAreaTextEntities[removedFloodProneAreaIndex]);
      this->floodProneAreas.erase(
        this->floodProneAreas.begin() + removedFloodProneAreaIndex);
      this->floodProneAreaEntities.erase(
        this->floodProneAreaEntities.begin() + removedFloodProneAreaIndex);
      this->floodProneAreaTextEntities.erase(
        this->floodProneAreaTextEntities.begin() + removedFloodProneAreaIndex);

      // Update the area IDs.
      for (int32_t i = 0; i < this->floodProneAreas.size(); i++) {
        if (this->registry.valid(this->floodProneAreaTextEntities[i])) {
          this->registry.patch<corex::core::Text>(
            this->floodProneAreaTextEntities[i],
            [&i](corex::core::Text& text) {
              text.setText(eastl::to_string(i));
            }
          );
        }
      }
    }

    if (removedLandslideProneAreaIndex != -1) {
      this->registry.destroy(
        this->landslideProneAreaEntities[removedLandslideProneAreaIndex]);
      this->registry.destroy(
        this->landslideProneAreaTextEntities[removedLandslideProneAreaIndex]);
      this->landslideProneAreas.erase(
        this->landslideProneAreas.begin() + removedLandslideProneAreaIndex);
      this->landslideProneAreaEntities.erase(
        this->landslideProneAreaEntities.begin()
        + removedLandslideProneAreaIndex);
      this->landslideProneAreaTextEntities.erase(
        this->landslideProneAreaTextEntities.begin()
        + removedLandslideProneAreaIndex);

      // Update the area IDs.
      for (int32_t i = 0; i < this->landslideProneAreas.size(); i++) {
        if (this->registry.valid(this->landslideProneAreaTextEntities[i])) {
          this->registry.patch<corex::core::Text>(
            this->landslideProneAreaTextEntities[i],
            [&i](corex::core::Text& text) {
              text.setText(eastl::to_string(i));
            }
          );
        }
      }
    }
  }

  void MainScene::buildWarningWindow()
  {
    if (!this->doesInputDataExist
        || !this->doesInputBoundingAreaFieldExist
        || !this->doesInputBuildingsExist
        || !this->doesGASettingsFieldExist
        || !this->doFloodProneAreasExist
        || !this->doLandslideProneAreasExist) {
      ImGui::Begin("Warnings");
      ImGui::BeginChild("Warnings List");

      int32_t numWarnings = 0;

      if (!this->doesInputDataExist) {
        ImGui::Text("WARNING: Input data does not exist.");
        numWarnings++;
      }

      if (!this->doesInputBoundingAreaFieldExist) {
        ImGui::Text("WARNING: Bounding area field in input data "
                    "does not exist.");
        numWarnings++;
      }

      if (!this->doesInputBuildingsExist) {
        ImGui::Text("WARNING: Input buildings field in input data "
                    "does not exist.");
        numWarnings++;
      }

      if (!this->doesGASettingsFieldExist) {
        ImGui::Text("WARNING: GA settings field in input data "
                    "does not exist.");
        numWarnings++;
      }

      if (!this->doFloodProneAreasExist) {
        ImGui::Text("WARNING: Flood-prone areas data in input data "
                    "does not exist.");
        numWarnings++;
      }

      if (!this->doLandslideProneAreasExist) {
        ImGui::Text("WARNING: Landslide-prone areas data in input data "
                    "does not exist.");
        numWarnings++;
      }

      ImGui::EndChild();

      ImGui::Text("Total No. of Warnings: %d", numWarnings);

      ImGui::End();
    }
  }

  void MainScene::buildInputBuildingsWindow()
  {
    static int32_t removedBuildingIndex;
    removedBuildingIndex = -1;

    ImGui::Begin("Input Buildings");

    if (ImGui::Button("Add another building")) {
      this->inputBuildings.push_back(InputBuilding{});

      // Add a column for the new input building for each pre-existing building.
      for (int32_t i = 0; i < this->inputBuildings.size() - 1; i++) {
        this->flowRates[i].push_back(1.f);
      }

      this->flowRates.push_back(
        eastl::vector<float>(this->inputBuildings.size(), 1.f));
      this->flowRates.back().back() = 0.f;

      assert(this->inputBuildings.size() == this->flowRates.size());
    }
    ImGui::Separator();

    ImGui::BeginChild("Input Buildings List");

    for (int32_t i = 0; i < this->inputBuildings.size(); i++) {
      ImGui::Separator();

      ImGui::Text("Building #%d", i);

      // Should be good enough for 9,999 input buildings.
      char lengthText[12];
      char widthText[11];
      char removeBuildingBtnText[21];

      sprintf(lengthText, "Length##%d", i);
      sprintf(widthText, "Width##%d", i);
      sprintf(removeBuildingBtnText, "Remove Building##%d", i);

      ImGui::InputFloat(lengthText, &(this->inputBuildings[i].length));
      ImGui::InputFloat(widthText, &(this->inputBuildings[i].width));

      if (ImGui::Button(removeBuildingBtnText)) {
        removedBuildingIndex = i;
      }
    }

    if (removedBuildingIndex != -1) {
      this->inputBuildings
           .erase(this->inputBuildings.begin() + removedBuildingIndex);

      this->flowRates.erase(this->flowRates.begin() + removedBuildingIndex);
      for (int32_t i = 0; i < this->flowRates.size(); i++) {
        this->flowRates[i].erase(
          this->flowRates[i].begin() + removedBuildingIndex);
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildFlowRateWindow()
  {
    ImGui::Begin("Flow Rate");
    ImGui::BeginChild("Flow Rate List");

    for (int32_t i = 0; i < this->inputBuildings.size(); i++) {
      ImGui::Separator();
      ImGui::Text("Building #%d", i);
      for (int32_t j = 0; j < this->inputBuildings.size(); j++) {
        if (i == j) {
          continue;
        }

        char buildingText[20];
        sprintf(buildingText, "Building #%d##%d", j, i);

        if (ImGui::InputFloat(buildingText, &(this->flowRates[i][j]))) {
          this->flowRates[j][i] = this->flowRates[i][j];
        }
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildGAControlsWindow()
  {
    ImGui::Begin("GA Controls");
    ImGui::BeginChild("GA Controls List");

    ImGui::InputFloat("Mutation Rate", &(this->gaSettings.mutationRate));
    ImGui::InputInt("Population Size", &(this->gaSettings.populationSize));
    ImGui::InputInt("No. of Generations", &(this->gaSettings.numGenerations));
    ImGui::InputInt("No. of Prev. Offsprings to Keep",
                    &(this->gaSettings.numPrevGenOffsprings));
    ImGui::InputFloat("Flood Penalty",
                      &(this->gaSettings.floodProneAreaPenalty));
    ImGui::InputFloat("Landslide Penalty",
                      &(this->gaSettings.landslideProneAreaPenalty));
    ImGui::InputFloat("Building Distance Weight",
                      &(this->gaSettings.buildingDistanceWeight));

    if (ImGui::BeginCombo("Selection Type",
                          castToCString(this->gaSettings.selectionType))) {
      // Gah. Let's hardcode the selection types for now. This is going
      // to be ugly.
      if (ImGui::Selectable(
            "Roulette Wheel",
            this->gaSettings.selectionType == SelectionType::RWS)) {
        this->gaSettings.selectionType = SelectionType::RWS;

        if (this->gaSettings.selectionType == SelectionType::RWS) {
          ImGui::SetItemDefaultFocus();
        }
      }

      if (ImGui::Selectable(
        "Tournament",
        this->gaSettings.selectionType == SelectionType::TS)) {
        this->gaSettings.selectionType = SelectionType::TS;

        if (this->gaSettings.selectionType == SelectionType::TS) {
          ImGui::SetItemDefaultFocus();
        }
      }

      ImGui::EndCombo();
    }

    if (this->gaSettings.selectionType == SelectionType::TS) {
      ImGui::InputInt("Tournament Size",
                      &(this->gaSettings.tournamentSize));
    }

    ImGui::Checkbox("Enable Local Search",
                    &(this->gaSettings.isLocalSearchEnabled));

    if (this->isGAThreadRunning) {
      ImGui::Text("Running GA... (Generation %d out of %d)",
                  this->geneticAlgo.getCurrentRunGenerationNumber() + 1,
                  this->gaSettings.numGenerations);
    } else {
      if (ImGui::Button("Generate Solution")) {
        this->isGAThreadRunning = true;

        std::thread gaThread{
          [this]() {
            // We need to copy the following vectors to this thread, because
            // they somehow get modified as the thread executes.
            auto inputBuildingsCopy = this->inputBuildings;
            auto boundingAreaCopy = this->boundingArea;
            auto flowRatesCopy = this->flowRates;
            auto floodProneAreasCopy = this->floodProneAreas;
            auto landslideProneAreasCopy = this->landslideProneAreas;

            this->solutions = this->geneticAlgo.generateSolutions(
              inputBuildingsCopy,
              boundingAreaCopy,
              flowRatesCopy,
              floodProneAreasCopy,
              landslideProneAreasCopy,
              this->gaSettings.mutationRate,
              this->gaSettings.populationSize,
              this->gaSettings.numGenerations,
              this->gaSettings.tournamentSize,
              this->gaSettings.numPrevGenOffsprings,
              this->gaSettings.floodProneAreaPenalty,
              this->gaSettings.landslideProneAreaPenalty,
              this->gaSettings.buildingDistanceWeight,
              this->gaSettings.isLocalSearchEnabled,
              this->gaSettings.selectionType
            );
            this->currSelectedGen = this->gaSettings.numGenerations;
            this->currSelectedGenSolution = 0;
            this->hasSolutionBeenSetup = false;

            this->recentGARunAvgFitnesses = this
                                              ->geneticAlgo
                                               .getRecentRunAverageFitnesses();
            this->recentGARunBestFitnesses = this
                                               ->geneticAlgo
                                                .getRecentRunBestFitnesses();
            this->recentGARunWorstFitnesses = this
                                                ->geneticAlgo
                                                 .getRecentRunWorstFitnesses();

            this->saveResultsToCSVFile();

            this->isGAThreadRunning = false;
          }
        };

        gaThread.detach();
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildGAResultsWindow()
  {
    ImGui::Begin("GA Results");
    ImGui::BeginChild("GA Results Content");

    ImGui::Text("Solution Fitness: %f",
                (this->currentSolution) ? this->currentSolution->getFitness()
                                        : 0.f);

    // Display combo box for selecting the generation and solution.
    if (ImGui::BeginCombo("Generation",
                          eastl::to_string(currSelectedGen).c_str())) {
      for (int32_t i = 0; i < this->solutions.size(); i++) {
        // Note that i = 0 is the generation with random solutions.
        const bool isItemSelected = (this->currSelectedGen == i);
        if (ImGui::Selectable(eastl::to_string(i).c_str(), isItemSelected)) {
          this->currSelectedGen = i;
          this->hasSolutionBeenSetup = false;
        }

        // Set the initial focus to this item when opening the combo box.
        if (isItemSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }

      ImGui::EndCombo();
    }

    if (ImGui::BeginCombo("Solution",
                          eastl::to_string(currSelectedGenSolution).c_str())) {
      if (this->solutions.size() > 0) {
        for (int32_t i = 0; i < this->solutions[currSelectedGen].size(); i++) {
          // Note that i = 0 is the generation with random solutions.
          const bool isItemSelected = (this->currSelectedGenSolution == i);
          if (ImGui::Selectable(eastl::to_string(i).c_str(), isItemSelected)) {
            this->currSelectedGenSolution = i;
            this->hasSolutionBeenSetup = false;
          }

          // Set the initial focus to this item when opening the combo box.
          if (isItemSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
      }

      ImGui::EndCombo();
    }

    ImGui::Columns(2, nullptr, false);

    if (ImGui::Button("Previous Generation")) {
      if (this->solutions.size() > 0) {
        this->currSelectedGen = corex::core::mod(
          this->currSelectedGen - 1,
          this->solutions.size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Next Generation")) {
      if (this->solutions.size() > 0) {
        this->currSelectedGen = corex::core::mod(
          this->currSelectedGen + 1,
          this->solutions.size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::NextColumn();

    if (ImGui::Button("Previous Solution")) {
      if (this->solutions.size() > 0) {
        this->currSelectedGenSolution = corex::core::mod(
          this->currSelectedGenSolution - 1,
          this->solutions[currSelectedGen].size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Next Solution")) {
      if (this->solutions.size() > 0) {
        this->currSelectedGenSolution = corex::core::mod(
          this->currSelectedGenSolution + 1,
          this->solutions[currSelectedGen].size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::Columns(1);

    if (this->solutions.size() > 0) {
      if (ImGui::Button(!(this->isGATimelinePlaying) ? "Play GA Timeline"
                                                     : "Pause GA Timeline")) {
        this->isGATimelinePlaying = !(this->isGATimelinePlaying);
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::SliderInt("Timeline Playback Speed",
                     &(this->gaTimelinePlaybackSpeed),
                     1,
                     4);

    ImGui::Separator();

    float avgFitnesses[this->recentGARunAvgFitnesses.size()];
    for (int32_t i = 0; i < this->recentGARunAvgFitnesses.size(); i++) {
      avgFitnesses[i] = this->recentGARunAvgFitnesses[i];
    }

    float bestFitnesses[this->recentGARunBestFitnesses.size()];
    for (int32_t i = 0; i < this->recentGARunBestFitnesses.size(); i++) {
      bestFitnesses[i] = this->recentGARunBestFitnesses[i];
    }

    float worstFitnesses[this->recentGARunWorstFitnesses.size()];
    for (int32_t i = 0; i < this->recentGARunWorstFitnesses.size(); i++) {
      worstFitnesses[i] = this->recentGARunWorstFitnesses[i];
    }

    ImPlot::SetNextPlotLimits(0.0, 1000, 0.0, 500.0);
    if (ImPlot::BeginPlot("Fitness Over Gen. ID", "Generation", "Fitness")) {
      if (this->showGAResultsAverage) {
        ImPlot::PlotLine("Average Fitness",
                         avgFitnesses,
                         this->recentGARunAvgFitnesses.size());
      }

      if (this->showGAResultsBest) {
        ImPlot::PlotLine("Best Fitness",
                         bestFitnesses,
                         this->recentGARunBestFitnesses.size());
      }

      if (this->showGAResultsWorst) {
        ImPlot::PlotLine("Worst Fitness",
                         worstFitnesses,
                         this->recentGARunWorstFitnesses.size());
      }

      ImPlot::EndPlot();
    }

    ImGui::Checkbox("Show Average Fitness", &(this->showGAResultsAverage));
    ImGui::Checkbox("Show Best Fitness", &(this->showGAResultsBest));
    ImGui::Checkbox("Show Worst Fitness", &(this->showGAResultsWorst));

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildDebugSolutionWindow()
  {
    ImGui::Begin("Solution Debugger");
    ImGui::BeginChild("Solution Debugger Content");

    float floatInputBuffer;

    if (this->hasSolutionBeenSetup && !this->isGATimelinePlaying) {
      for (int32_t i = 0; i < this->currentSolution->getNumBuildings(); i++) {
        ImGui::Text("Building #%d", i);

        // Should be good enough for 9,999 input buildings.
        char xPosText[8];
        char yPosText[8];
        char angleText[12];

        sprintf(xPosText, "X##%d", i);
        sprintf(yPosText, "Y##%d", i);
        sprintf(angleText, "Angle##%d", i);

        floatInputBuffer = this->currentSolution->getBuildingXPos(i);
        if (ImGui::InputFloat(xPosText, &floatInputBuffer)) {
          this->currentSolution->setBuildingXPos(i, floatInputBuffer);
          this->hasSolutionBeenSetup = false;
        }

        floatInputBuffer = this->currentSolution->getBuildingYPos(i);
        if (ImGui::InputFloat(yPosText, &floatInputBuffer)) {
          this->currentSolution->setBuildingYPos(i, floatInputBuffer);
          this->hasSolutionBeenSetup = false;
        }

        floatInputBuffer = this->currentSolution->getBuildingRotation(i);
        if (ImGui::InputFloat(angleText, &floatInputBuffer)) {
          this->currentSolution->setBuildingRotation(i, floatInputBuffer);
          this->hasSolutionBeenSetup = false;
        }
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::handleGATimelinePlayback(float timeDelta)
  {
    static float timeElapsed = 0.f;
    if (this->isGATimelinePlaying) {
      timeElapsed += timeDelta;

      const float timePerGeneration = 0.15f / this->gaTimelinePlaybackSpeed;
      if (corex::core::floatGreEqual(timeElapsed, timePerGeneration)) {
        this->currSelectedGen = corex::core::mod(
          this->currSelectedGen + 1,
          this->solutions.size());
        this->hasSolutionBeenSetup = false;

        timeElapsed = 0.f;
      }
    } else {
      timeElapsed = 0.f;
    }
  }

  void MainScene::handleWindowEvents(const corex::core::WindowEvent& e)
  {
    if (e.event.window.event == SDL_WINDOWEVENT_CLOSE) {
      this->setSceneStatus(corex::core::SceneStatus::DONE);
    }
  }

  void MainScene::handleKeyboardEvents(const corex::core::KeyboardEvent& e)
  {
    if (e.keyState == corex::core::KeyState::KEY_DOWN) {
      switch (e.keyCode) {
        case SDLK_w:
          this->camera.moveY(
            -corex::core::metersToPixels(this->cameraMovementSpeed,
                                         this->getPPMRatio())
            * this->timeDelta
          );          
          break;
        case SDLK_a:
          this->camera.moveX(
            -corex::core::metersToPixels(this->cameraMovementSpeed,
                                         this->getPPMRatio())
            * this->timeDelta
          );
          break;
        case SDLK_s:
          this->camera.moveY(
            corex::core::metersToPixels(this->cameraMovementSpeed,
                                        this->getPPMRatio())
            * this->timeDelta
          );
          break;
        case SDLK_d:
          this->camera.moveX(
            corex::core::metersToPixels(this->cameraMovementSpeed,
                                        this->getPPMRatio())
            * this->timeDelta
          );
          break;
        case SDLK_TAB:
          if (e.numRepeats == 0) {
            this->needUpdateBuildingRenderMode = true;
          }
          break;
      }
    }
  }

  void
  MainScene::handleMouseButtonEvents(const corex::core::MouseButtonEvent& e)
  {
    if (e.buttonType == corex::core::MouseButtonType::MOUSE_BUTTON_LEFT
        && e.buttonState == corex::core::MouseButtonState::MOUSE_BUTTON_DOWN
        && e.numRepeats == 0) {
      if (this->currentContext == Context::DRAW_AREA_BOUND
          || this->currentContext == Context::DRAW_FLOOD_PRONE_AREA
          || this->currentContext == Context::DRAW_LANDSLIDE_PRONE_AREA) {
        corex::core::LineSegments *targetWIPArea = nullptr;
        switch (this->currentContext) {
          case Context::DRAW_AREA_BOUND:
            targetWIPArea = &(this->wipBoundingArea);
            break;
          case Context::DRAW_FLOOD_PRONE_AREA:
          case Context::DRAW_LANDSLIDE_PRONE_AREA:
            targetWIPArea = &(this->wipHazardArea);
            break;
          default:
            break;
        }

        if (this->isCloseAreaTriggerEnabled) {
          int32_t lastElementIndex = targetWIPArea->vertices.size() - 1;
          float dist = corex::core::distance2D(
            targetWIPArea->vertices[0],
            targetWIPArea->vertices[lastElementIndex]);
          if (corex::core::floatLessEqual(dist, 7.f)) {
            // Alright, we're done drawing the area.
            switch (this->currentContext) {
              case Context::DRAW_FLOOD_PRONE_AREA:
                this->floodProneAreas.push_back();
                break;
              case Context::DRAW_LANDSLIDE_PRONE_AREA:
                this->landslideProneAreas.push_back();
                break;
              default:
                break;
            }

            for (int32_t i = 0; i < targetWIPArea->vertices.size() - 1; i++) {
              // Note: The last elements contains the position of the mouse.
              //       We shouldn't add it, otherwise we'd get another edge
              //       to the bounding/hazard area that we don't want.
              switch (this->currentContext) {
                case Context::DRAW_AREA_BOUND:
                  this->boundingArea.vertices.push_back(
                    targetWIPArea->vertices[i]);
                  break;
                case Context::DRAW_FLOOD_PRONE_AREA:
                  this->floodProneAreas.back().vertices.push_back(
                    targetWIPArea->vertices[i]);
                  break;
                case Context::DRAW_LANDSLIDE_PRONE_AREA:
                  this->landslideProneAreas.back().vertices.push_back(
                    targetWIPArea->vertices[i]);
                  break;
                default:
                  break;
              }
            }

            switch (this->currentContext) {
              case Context::DRAW_AREA_BOUND:
                this->boundingAreaTriangles = cx::earClipTriangulate(
                  this->boundingArea);
                break;
              case Context::DRAW_FLOOD_PRONE_AREA:
                this->floodProneAreaEntities.push_back(entt::null);
                this->floodProneAreaTextEntities.push_back(entt::null);
                break;
              case Context::DRAW_LANDSLIDE_PRONE_AREA:
                this->landslideProneAreaEntities.push_back(entt::null);
                this->landslideProneAreaTextEntities.push_back(entt::null);
                break;
              default:
                break;
            }

            this->currentContext = Context::NO_ACTION;
            this->isCloseAreaTriggerEnabled = false;
            this->closeAreaTriggerCircle.position = corex::core::Point{
              0.f, 0.f
            };

            targetWIPArea->vertices.clear();
            return;
          }
        }

        // Once the number of vertices the WIP bounding/hazard area has becomes
        // two, the Point instance we're pushing will hold the current position
        // of the mouse cursor, while we're drawing the area.
        targetWIPArea->vertices.push_back(
          corex::core::screenToWorldCoordinates(
            corex::core::Point{
              static_cast<float>(e.x),
              static_cast<float>(e.y)
            },
            this->camera
          )
        );

        if (targetWIPArea->vertices.size() == 1) {
          targetWIPArea->vertices.push_back(
            corex::core::screenToWorldCoordinates(
              corex::core::Point{
                static_cast<float>(e.x),
                static_cast<float>(e.y)
              },
              this->camera
            )
          );
        }

        if (targetWIPArea->vertices.size() == 4) {
          this->closeAreaTriggerCircle.position = targetWIPArea->vertices[0];
          this->isCloseAreaTriggerEnabled = true;
        }
      }
    } else if (e.buttonType == corex::core::MouseButtonType::MOUSE_BUTTON_RIGHT
               && e.buttonState == corex::core::MouseButtonState
                                              ::MOUSE_BUTTON_DOWN
               && e.numRepeats == 0) {
      this->currentContext = Context::NO_ACTION;
      this->isCloseAreaTriggerEnabled = false;
      this->closeAreaTriggerCircle.position = corex::core::Point{ 0.f, 0.f };

      if (this->currentContext == Context::DRAW_AREA_BOUND) {
        this->wipBoundingArea.vertices.clear();
      } else if (this->currentContext != Context::NO_ACTION) {
        this->wipHazardArea.vertices.clear();
      }
    }
  }

  void
  MainScene::handleMouseMovementEvents(const corex::core::MouseMovementEvent& e)
  {
    if (this->currentContext == Context::DRAW_AREA_BOUND
        || this->currentContext == Context::DRAW_FLOOD_PRONE_AREA
        || this->currentContext == Context::DRAW_LANDSLIDE_PRONE_AREA) {
      corex::core::LineSegments* targetWIPArea = nullptr;
      switch (this->currentContext) {
        case Context::DRAW_AREA_BOUND:
          targetWIPArea = &(this->wipBoundingArea);
          break;
        case Context::DRAW_FLOOD_PRONE_AREA:
        case Context::DRAW_LANDSLIDE_PRONE_AREA:
          targetWIPArea = &(this->wipHazardArea);
          break;
        default:
          break;
      }

      if (targetWIPArea) {
        if (!targetWIPArea->vertices.empty()) {
          int32_t lastElementIndex = targetWIPArea->vertices.size() - 1;
          targetWIPArea->vertices[lastElementIndex] =
            corex::core::screenToWorldCoordinates(
              corex::core::Point{
                static_cast<float>(e.x),
                static_cast<float>(e.y)
              },
              this->camera
            );
        }

        if (this->isCloseAreaTriggerEnabled) {
          int32_t lastElementIndex = targetWIPArea->vertices.size() - 1;
          float dist = corex::core::distance2D(
            targetWIPArea->vertices[0],
            targetWIPArea->vertices[lastElementIndex]);
          if (corex::core::floatLessEqual(dist, 7.f)) {
            this->closeAreaTriggerCircle.radius = 5.f;
          } else {
            this->closeAreaTriggerCircle.radius = 0.f;
          }
        }
      }
    }
  }

  void
  MainScene::handleMouseScrollEvents(const corex::core::MouseScrollEvent& e)
  {
    if (e.yScrollAmount > 0) {
      // Scroll up.
      this->camera.zoom(corex::core::CameraZoomState::IN);
    } else if (e.yScrollAmount < 0) {
      // Scroll down.
      this->camera.zoom(corex::core::CameraZoomState::OUT);
    }
  }

  void MainScene::saveResultsToCSVFile()
  {
    auto time = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    std::stringstream resultsFileRelPath;
    resultsFileRelPath << "data/results-"
                       << std::to_string(time)
                       << ".csv";

    std::filesystem::path resultsFilePath = corex::core::getBinFolder()
                                            / resultsFileRelPath.str();
    std::ofstream resultsFile;
    resultsFile.open(resultsFilePath.string());

    // Put the header of the CSV
    for (int32_t i = 0; i < this->gaSettings.numGenerations; i++) {
      resultsFile << ", Gen. #" << i;
    }
    resultsFile << "\n";

    resultsFile << "Average Fitness";
    for (float& fitness : this->recentGARunAvgFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "Best Fitness";
    for (float& fitness : this->recentGARunBestFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "Worst Fitness";
    for (float& fitness : this->recentGARunWorstFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "\n";
    resultsFile << "GA Parameters\n"
                << "Mutation Rate:," << this->gaSettings.mutationRate
                << "\n"
                << "Population Size:," << this->gaSettings.populationSize
                << "\n"
                << "No. of Generations:," << this->gaSettings.numGenerations
                << "\n"
                << "Tournament Size:," << this->gaSettings.tournamentSize
                << "\n"
                << "Flood Penalty:," << this->gaSettings.floodProneAreaPenalty
                << "\n"
                << "Landslide Penalty:,"
                << this->gaSettings.landslideProneAreaPenalty
                << "\n"
                << "Building Distance Weight:,"
                << this->gaSettings.buildingDistanceWeight
                << "\n"
                << "Local Search:,"
                << ((this->gaSettings.isLocalSearchEnabled) ? "Enabled"
                                                            : "Disabled")
                << "\n";

    resultsFile << "\n";
    resultsFile << "Input Buildings" << "\n"
                << "Width,Length" << "\n";
    for (int32_t i = 0; i < this->inputBuildings.size(); i++) {
      resultsFile << this->inputBuildings[i].width << ","
                  << this->inputBuildings[i].length << "\n";
    }

    resultsFile << "\n";
    resultsFile << "Best Solution:" << "\n"
                << "x,y,Rotation" << "\n";
    for (int32_t i = 0; i < this->solutions.back()[0].getNumBuildings(); i++) {
      resultsFile << this->solutions.back()[0].getBuildingXPos(i) << ","
                  << this->solutions.back()[0].getBuildingYPos(i) << ","
                  << this->solutions.back()[0].getBuildingRotation(i) << "\n";
    }

    resultsFile.close();
  }

  void MainScene::clearCurrentlyRenderedSolution()
  {
    for (entt::entity& e : this->buildingEntities) {
      this->registry.destroy(e);
    }

    for (entt::entity& e : this->buildingTextEntities) {
      this->registry.destroy(e);
    }

    this->buildingEntities.clear();
    this->buildingTextEntities.clear();
  }
}
