#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
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
#include <bpt/evaluator.hpp>
#include <bpt/gui.hpp>
#include <bpt/json_specializations.hpp>
#include <bpt/MainScene.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/Result.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Time.hpp>

namespace bpt
{
  MainScene::MainScene(entt::registry& registry,
                       entt::dispatcher& eventDispatcher,
                       corex::core::AssetManager& assetManager,
                       corex::core::Camera& camera)
    : currentContext(Context::NO_ACTION)
    , geneticAlgo()
    , gwoAlgo()
    , psoAlgo()
    , hillClimbingAlgo()
    , currentAlgorithm(AlgorithmType::GA)
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
        CrossoverType::NONE,
        SelectionType::NONE,
        true
      })
    , lsSettings({
        0.0
      })
    , settings("algorithm_settings.bptstg")
    , currentSolution(nullptr)
    , solutions()
    , solutionBuffer()
    , isAlgoThreadRunning(false)
    , hasSolutionBeenSetup(false)
    , doesInputDataExist(false)
    , doesInputBoundingAreaFieldExist(false)
    , doesInputBuildingsExist(false)
    , doesGASettingsFieldExist(false)
    , doFloodProneAreasExist(false)
    , doLandslideProneAreasExist(false)
    , isCloseAreaTriggerEnabled(false)
    , showResultsAverage(true)
    , showResultsBest(true)
    , showResultsWorst(true)
    , needUpdateBuildingRenderMode(false)
    , isResultsTimelinePlaying(false)
    , closeAreaTriggerCircle(corex::core::Circle{
        corex::core::Point{ 0.f, 0.f }, 0.0f
      })
    , cameraMovementSpeed(15.f)
    , timeDelta(0.f)
    , timePerIteration(0.15f)
    , currSelectedIter(0)
    , currSelectedIterSolution(0)
    , resultsTimelinePlaybackSpeed(1)
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
    , gwoPreyEntities()
    , flowRates()
    , buildingTextEntities()
    , floodProneAreaTextEntities()
    , landslideProneAreaTextEntities()
    , recentRunAvgFitnesses()
    , recentRunBestFitnesses()
    , recentRunWorstFitnesses()
    , recentRunElapsedTime(0.0)
    , inputData()
    , areNewSolutionsReady()
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
            this->gaSettings.crossoverType =
              gaSettingsJSON["crossoverType"].get<CrossoverType>();
            this->gaSettings.selectionType =
              gaSettingsJSON["selectionType"].get<SelectionType>();
            this->gaSettings.keepInfeasibleSolutions =
              gaSettingsJSON["keepInfeasibleSolutions"].get<bool>();

            this->doesGASettingsFieldExist = true;
          }

          if (this->inputData.contains("lsSettings")) {
            nlohmann::json lsSettingsJSON = this->inputData["lsSettings"];
            this->lsSettings.timeLimit =
              lsSettingsJSON["timeLimit"].get<double>();
          }
        }
      }
    }

    // Give GWO settings default values if they don't exist.
    cx::ReturnState returnState;

    // General Settings
    returnState = this->settings.getFloatVariable("boundingAreaLength")
                                .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("boundingAreaLength", 450.f);
    }

    returnState = this->settings.getFloatVariable("boundingAreaHeight")
                                .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("boundingAreaHeight", 250.f);
    }

    // GA Settings
    returnState = this->settings
                       .getIntegerVariable("gaNumLSIters")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gaNumLSIters", 300);
    }

    // GWO Settings
    returnState = this->settings.getIntegerVariable("gwoNumWolves").returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoNumWolves", 30);
    }

    returnState = this->settings
                       .getIntegerVariable("gwoNumIterations")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoNumIterations", 1000);
    }

    returnState = this->settings
                       .getFloatVariable("gwoAlphaDecayRate")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoAlphaDecayRate", 0.01f);
    }

    returnState = this->settings
      .getFloatVariable("gwoCValue")
      .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoCValue", 4.f);
    }

    returnState = this->settings
                       .getFloatVariable("gwoFloodPenalty")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoFloodPenalty", 10000.f);
    }

    returnState = this->settings
                       .getFloatVariable("gwoLandslidePenalty")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoLandslidePenalty", 25000.f);
    }

    returnState = this->settings
                       .getFloatVariable("gwoBuildingDistanceWeight")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoBuildingDistanceWeight", 1.f);
    }

    returnState = this->settings
                       .getBooleanVariable("gwoKeepInfeasibleSolutions")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoKeepInfeasibleSolutions", true);
    }

    returnState = this->settings
                       .getBooleanVariable("gwoIsLocalSearchEnabled")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoIsLocalSearchEnabled", true);
    }

    returnState = this->settings
                       .getIntegerVariable("gwoNumLSIters")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("gwoNumLSIters", 300);
    }

    // PSO Settings
    returnState = this->settings
                       .getIntegerVariable("psoNumIterations")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("psoNumIterations", 400);
    }

    returnState = this->settings
                       .getIntegerVariable("psoNumParticles")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("psoNumParticles", 50);
    }

    returnState = this->settings
                       .getFloatVariable("psoW")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("psoW", 0.05f);
    }

    returnState = this->settings
                       .getFloatVariable("psoC1")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("psoC1", 2.f);
    }

    returnState = this->settings
                       .getFloatVariable("psoC2")
                       .returnState;
    if (returnState == cx::ReturnState::RETURN_FAIL) {
      this->settings.setVariable("psoC2", 2.f);
    }
  }

  void MainScene::update(float timeDelta)
  {
    this->timeDelta = timeDelta;

    this->handleGATimelinePlayback(timeDelta);

    if (this->areNewSolutionsReady) {
      this->solutions = this->solutionBuffer;
      this->areNewSolutionsReady = false;
    }

    if (this->solutions.size() > 0) {
      this->currentSolution = &(this->solutions[this->currSelectedIter]
                                               [this->currSelectedIterSolution]);
    }

    if (this->currentSolution
        && !this->hasSolutionBeenSetup
        && !this->isAlgoThreadRunning) {
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
          this->currentSolution->getBuildingAngle(i),
          SDL_Color{0, 102, 51, 255},
          true);

        this->buildingEntities.push_back(e);
      }

//      for (int32_t i = 0; i < this->currentSolution->getNumBuildings(); i++) {
//        // We are using two separate loops to update the building and building
//        // text entities for performance reasons. Doing this will ensure that
//        // there will be more cache hits than cache misses.
//        entt::entity e = this->registry.create();
//        this->registry.emplace<corex::core::Position>(
//          e,
//          this->currentSolution->getBuildingXPos(i),
//          this->currentSolution->getBuildingYPos(i),
//          0.f,
//          static_cast<int8_t>(5));
//        this->registry.emplace<corex::core::Renderable>(
//          e,
//          corex::core::RenderableType::TEXT);
//        this->registry.emplace<corex::core::Text>(
//          e,
//          eastl::to_string(i),
//          this->assetManager.getFont("liberation-sans-regular-font", 15),
//          SDL_Color{ 255, 255, 255, 255 });
//
//        this->buildingTextEntities.push_back(e);
//      }

      this->currentSolution->setFitness(computeSolutionFitness(
        *(this->currentSolution),
        this->inputBuildings,
        this->boundingArea,
        this->flowRates,
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

    this->buildConstructBoundingAreaWindow();
    this->buildInputBuildingsWindow();
    this->buildFlowRateWindow();
    this->buildAlgorithmControlsWindow();
    this->buildAlgorithmResultsWindow();
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
      this->inputData["gaSettings"]["crossoverType"] =
        this->gaSettings.crossoverType;
      this->inputData["gaSettings"]["selectionType"] =
        this->gaSettings.selectionType;
      this->inputData["gaSettings"]["keepInfeasibleSolutions"] =
        this->gaSettings.keepInfeasibleSolutions;

      this->inputData["lsSettings"] = nlohmann::json::object();
      this->inputData["lsSettings"]["timeLimit"] = this->lsSettings.timeLimit;

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

    this->settings.save();
  }

  void MainScene::buildConstructBoundingAreaWindow()
  {
    ImGui::Begin("Bounding Area");

    static bool isFirstRun = true;
    static float areaLength = this->settings
                                   .getFloatVariable("boundingAreaLength")
                                   .value;
    static float areaHeight = this->settings
                                   .getFloatVariable("boundingAreaHeight")
                                   .value;
    static float prevAreaLength = areaLength;
    static float prevAreaHeight = areaHeight;

    ImGui::InputFloat("Length", &areaLength);
    ImGui::InputFloat("Height", &areaHeight);

    if (isFirstRun
        || (areaLength != prevAreaLength)
        || (areaHeight != prevAreaHeight)) {
      cx::Point startingPoint{ 734.f, 11.f };
      cx::Polygon<4> boundingAreaRect = cx::createRectangle(startingPoint,
                                                            areaLength,
                                                            areaHeight);
      this->boundingArea.vertices = eastl::vector<cx::Point>(
        boundingAreaRect.vertices.begin(),
        boundingAreaRect.vertices.end());
      this->boundingAreaTriangles = cx::earClipTriangulate(this->boundingArea);

      prevAreaLength = areaLength;
      prevAreaHeight = areaHeight;
    }

    ImGui::Text("Bound Area Vertices");
    ImGui::BeginChild("Vertices List");
    for (corex::core::Point& pt : this->boundingArea.vertices) {
      ImGui::Text("%f, %f", pt.x, pt.y);
    }
    ImGui::EndChild();

    ImGui::End();

    this->settings.setVariable("boundingAreaLength", areaLength);
    this->settings.setVariable("boundingAreaHeight", areaHeight);
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

        ImGui::InputFloat(buildingText, &(this->flowRates[i][j]));
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildAlgorithmControlsWindow()
  {
    ImGui::Begin("Algorithm Controls");
    ImGui::BeginChild("Algorithms Controls List");

    // TODO: Implement templated function that computes enum length.
    constexpr int32_t numAlgoItems = 4;
    static const AlgorithmType algoItems[numAlgoItems] = {
      AlgorithmType::GA, AlgorithmType::GWO,
      AlgorithmType::PSO, AlgorithmType::HC
    };
    drawComboBox("Algorithm", this->currentAlgorithm, algoItems);

    ImGui::Separator();

    switch (this->currentAlgorithm) {
      case AlgorithmType::GA: {
        static int32_t numLSIters =
          this->settings.getIntegerVariable("gaNumLSIters").value;

        ImGui::InputFloat("Mutation Rate",
                          &(this->gaSettings.mutationRate));
        ImGui::InputInt("Population Size",
                        &(this->gaSettings.populationSize));
        ImGui::InputInt("No. of Generations",
                        &(this->gaSettings.numGenerations));
        ImGui::InputInt("No. of Prev. Offsprings to Keep",
                        &(this->gaSettings.numPrevGenOffsprings));
        ImGui::InputFloat("Flood Penalty",
                          &(this->gaSettings.floodProneAreaPenalty));
        ImGui::InputFloat("Landslide Penalty",
                          &(this->gaSettings.landslideProneAreaPenalty));
        ImGui::InputFloat("Building Distance Weight",
                          &(this->gaSettings.buildingDistanceWeight));

        constexpr int32_t numSelectionItems = 4;
        static const SelectionType selectionItems[numSelectionItems] = {
          SelectionType::NONE,
          SelectionType::TS,
          SelectionType::RWS,
          SelectionType::RS
        };
        drawComboBox("Selection Type",
                     this->gaSettings.selectionType,
                     selectionItems);

        constexpr int32_t numCrossoverItems = 4;
        static const CrossoverType crossoverItems[numCrossoverItems] = {
          CrossoverType::NONE,
          CrossoverType::UNIFORM,
          CrossoverType::BOX,
          CrossoverType::ARITHMETIC
        };
        drawComboBox("Crossover Type",
                     this->gaSettings.crossoverType,
                     crossoverItems);

        if (this->gaSettings.selectionType == SelectionType::TS) {
          ImGui::InputInt("Tournament Size",
                          &(this->gaSettings.tournamentSize));
        }

        ImGui::Checkbox("Keep Infeasible Solutions",
                        &(this->gaSettings.keepInfeasibleSolutions));
        ImGui::Checkbox("Enable Local Search",
                        &(this->gaSettings.isLocalSearchEnabled));

        if (this->gaSettings.isLocalSearchEnabled) {
          ImGui::InputInt("No. of LS Iterations", &numLSIters);
        }

        this->settings.setVariable("gaNumLSIters", numLSIters);
      } break;
      case AlgorithmType::GWO: {
        static int32_t numWolves =
          this->settings.getIntegerVariable("gwoNumWolves").value;
        static int32_t numIterations =
          this->settings.getIntegerVariable("gwoNumIterations").value;
        static float alphaDecayRate =
          this->settings.getFloatVariable("gwoAlphaDecayRate").value;
        static float cValue =
          this->settings.getFloatVariable("gwoCValue").value;
        static float floodProneAreasPenalty =
          this->settings.getFloatVariable("gwoFloodPenalty").value;
        static float landslideProneAreasPenalty =
          this->settings.getFloatVariable("gwoLandslidePenalty").value;
        static float buildingDistanceWeight =
          this->settings.getFloatVariable("gwoBuildingDistanceWeight").value;
        static bool keepInfeasibleSolutions =
          this->settings.getBooleanVariable("gwoKeepInfeasibleSolutions").value;
        static bool isLocalSearchEnabled =
          this->settings.getBooleanVariable("gwoIsLocalSearchEnabled").value;
        static int32_t numLSIters =
          this->settings.getIntegerVariable("gwoNumLSIters").value;

        ImGui::InputInt("No. of Wolves", &numWolves);
        ImGui::InputInt("No. of Iterations", &numIterations);
        ImGui::SliderFloat("Alpha Decay Rate", &alphaDecayRate, 0.f, 1.f);
        ImGui::InputFloat("C Value", &cValue);
        ImGui::InputFloat("Flood Penalty", &floodProneAreasPenalty);
        ImGui::InputFloat("Landslide Penalty", &landslideProneAreasPenalty);
        ImGui::InputFloat("Building Distance Weight", &buildingDistanceWeight);
        ImGui::Checkbox("Keep Infeasible Solutions", &keepInfeasibleSolutions);
        ImGui::Checkbox("Enable Local Search", &isLocalSearchEnabled);

        if (isLocalSearchEnabled) {
          ImGui::InputInt("No. of LS Iterations", &numLSIters);
        }

        this->settings.setVariable("gwoNumWolves", numWolves);
        this->settings.setVariable("gwoNumIterations", numIterations);
        this->settings.setVariable("gwoAlphaDecayRate", alphaDecayRate);
        this->settings.setVariable("gwoCValue", cValue);
        this->settings.setVariable("gwoFloodPenalty", floodProneAreasPenalty);
        this->settings.setVariable("gwoLandslidePenalty",
                                   landslideProneAreasPenalty);
        this->settings.setVariable("gwoBuildingDistanceWeight",
                                   buildingDistanceWeight);
        this->settings.setVariable("gwoKeepInfeasibleSolutions",
                                   keepInfeasibleSolutions);
        this->settings.setVariable("gwoIsLocalSearchEnabled",
                                   isLocalSearchEnabled);
        this->settings.setVariable("gwoNumLSIters", numLSIters);
      } break;
      case AlgorithmType::PSO: {
        static int32_t numParticles =
          this->settings.getIntegerVariable("psoNumParticles").value;
        static int32_t numIterations =
          this->settings.getIntegerVariable("psoNumIterations").value;
        static float w = this->settings.getFloatVariable("psoW").value;
        static float c1 = this->settings.getFloatVariable("psoC1").value;
        static float c2 = this->settings.getFloatVariable("psoC2").value;

        ImGui::InputInt("No. of Particles", &numParticles);
        ImGui::InputInt("No. of Iterations", &numIterations);
        ImGui::InputFloat("w", &w);
        ImGui::InputFloat("c1", &c1);
        ImGui::InputFloat("c2", &c2);

        this->settings.setVariable("psoNumParticles", numParticles);
        this->settings.setVariable("psoNumIterations", numIterations);
        this->settings.setVariable("psoW", w);
        this->settings.setVariable("psoC1", c1);
        this->settings.setVariable("psoC2", c2);
      } break;
      case AlgorithmType::HC: {
        ImGui::Text("WIP");
      } break;
    }

    if (this->isAlgoThreadRunning) {
      switch (this->currentAlgorithm) {
        case AlgorithmType::GA: {
          if (this->gaSettings.isLocalSearchEnabled) {
            int32_t numLSIters = this->settings
                                      .getIntegerVariable("gaNumLSIters")
                                      .value;
            ImGui::Text("Progress: Generation %d out of %d + %d",
                        this->geneticAlgo.getCurrentRunIterationNumber() + 1,
                        this->gaSettings.numGenerations,
                        numLSIters);
          } else {
            ImGui::Text("Progress: Generation %d out of %d",
                        this->geneticAlgo.getCurrentRunIterationNumber() + 1,
                        this->gaSettings.numGenerations);
          }
        } break;
        case AlgorithmType::GWO: {
          bool isLSEnabled = this->settings
                                  .getBooleanVariable("gwoIsLocalSearchEnabled")
                                  .value;
          int32_t numLSIters = this->settings
                                    .getIntegerVariable("gwoNumLSIters")
                                    .value;
          if (isLSEnabled) {
            ImGui::Text("Progress: Iteration %d out of %d + %d",
                        this->gwoAlgo.getCurrentRunIterationNumber() + 1,
                        this->settings.getIntegerVariable("gwoNumIterations")
                             .value,
                        numLSIters);
          } else {
            ImGui::Text("Progress: Iteration %d out of %d",
                        this->gwoAlgo.getCurrentRunIterationNumber() + 1,
                        this->settings.getIntegerVariable("gwoNumIterations")
                             .value);
          }
        } break;
        case AlgorithmType::PSO: {
          ImGui::Text("Progress: Iteration %d out of %d",
                      this->psoAlgo.getCurrentRunIterationNumber() + 1,
                      this->settings.getIntegerVariable("psoNumIterations")
                           .value);
        } break;
      }
    } else {
      if (ImGui::Button("Generate Solution")) {
        this->isAlgoThreadRunning = true;
        this->areNewSolutionsReady = false;

        switch (this->currentAlgorithm) {
          case AlgorithmType::GA: {
            std::thread gaThread{
              [this]() {
                // We need to copy the following vectors to this thread, because
                // they somehow get modified as the thread executes.
                auto inputBuildingsCopy = this->inputBuildings;
                auto boundingAreaCopy = this->boundingArea;
                auto flowRatesCopy = this->flowRates;
                auto floodProneAreasCopy = this->floodProneAreas;
                auto landslideProneAreasCopy = this->landslideProneAreas;

                GAResult result = this->geneticAlgo.generateSolutions(
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
                  this->gaSettings.crossoverType,
                  this->gaSettings.selectionType,
                  this->settings.getIntegerVariable("gaNumLSIters").value,
                  this->gaSettings.keepInfeasibleSolutions
                );

                std::cout << "Finished optimizing!\n";

                this->recentRunAvgFitnesses = result.averageFitnesses;
                this->recentRunBestFitnesses = result.bestFitnesses;
                this->recentRunWorstFitnesses = result.worstFitnesses;
                this->recentRunElapsedTime = result.elapsedTime;
                this->solutionBuffer = result.solutions;

                auto time = std::chrono::system_clock::to_time_t(
                  std::chrono::system_clock::now());
                std::stringstream resultsFileRelPath;
                resultsFileRelPath << "ga-results-" << std::to_string(time);
                result.saveToCSV(cx::stdStrToEAStr(resultsFileRelPath.str()));

                // NOTE: There's a weird bug that occurs when we don't subtract
                //       the solution size by one when assigning the value o
                //       the currently selected generation.
                //
                //       When the GA is run without local search, the program
                //       will not show any buildings upon completion of the
                //       optimization. Clicking "Next Generation" or
                //       "Previous Generation" will result in the wrong
                //       generation index from being removed from the list of
                //       generations. If local search is run, the program
                //       crashes instead.
                //
                //       I could fix this now, but it's not a priority. Too bad.
                this->currSelectedIter = this->solutionBuffer.size() - 1;
                this->currSelectedIterSolution = 0;
                this->hasSolutionBeenSetup = false;
                this->areNewSolutionsReady = true;

                this->isAlgoThreadRunning = false;
              }
            };

            gaThread.detach();
          } break;
          case AlgorithmType::GWO: {
            std::thread gwoThread{
              [this]() {
                // We need to copy the following vectors to this thread, because
                // they somehow get modified as the thread executes.
                auto inputBuildingsCopy = this->inputBuildings;
                auto boundingAreaCopy = this->boundingArea;
                auto flowRatesCopy = this->flowRates;
                auto floodProneAreasCopy = this->floodProneAreas;
                auto landslideProneAreasCopy = this->landslideProneAreas;

                GWOResult result = this->gwoAlgo.generateSolutions(
                  inputBuildingsCopy,
                  boundingAreaCopy,
                  flowRatesCopy,
                  floodProneAreasCopy,
                  landslideProneAreasCopy,
                  this->settings.getIntegerVariable("gwoNumWolves").value,
                  this->settings.getIntegerVariable("gwoNumIterations").value,
                  this->settings.getFloatVariable("gwoCValue").value,
                  this->settings.getFloatVariable("gwoAlphaDecayRate").value,
                  this->settings.getFloatVariable("gwoFloodPenalty").value,
                  this->settings.getFloatVariable("gwoLandslidePenalty").value,
                  this->settings.getFloatVariable("gwoBuildingDistanceWeight")
                       .value,
                  this->settings.getBooleanVariable("gwoIsLocalSearchEnabled")
                       .value,
                  this->settings.getIntegerVariable("gwoNumLSIters").value,
                  this->settings
                       .getBooleanVariable("gwoKeepInfeasibleSolutions")
                       .value);

                std::cout << "Finished optimizing!\n";

                this->solutionBuffer = result.solutions;
                this->recentRunAvgFitnesses = eastl::vector<double>(
                  result.averageFitnesses);
                this->recentRunBestFitnesses = eastl::vector<double>(
                  result.bestFitnesses);
                this->recentRunWorstFitnesses = eastl::vector<double>(
                  result.worstFitnesses);
                this->recentRunElapsedTime = result.elapsedTime;

                auto time = std::chrono::system_clock::to_time_t(
                  std::chrono::system_clock::now());
                std::stringstream resultsFileRelPath;
                resultsFileRelPath << "gwo-results-" << std::to_string(time);
                result.saveToCSV(cx::stdStrToEAStr(resultsFileRelPath.str()));

                // NOTE: There's a weird bug that occurs when we don't subtract
                //       the solution size by one when assigning the value o
                //       the currently selected generation.
                //
                //       When the GA is run without local search, the program
                //       will not show any buildings upon completion of the
                //       optimization. Clicking "Next Generation" or
                //       "Previous Generation" will result in the wrong
                //       generation index from being removed from the list of
                //       generations. If local search is run, the program
                //       crashes instead.
                //
                //       I could fix this now, but it's not a priority. Too bad.
                this->currSelectedIter = this->solutionBuffer.size() - 1;
                this->currSelectedIterSolution = 0;
                this->hasSolutionBeenSetup = false;
                this->areNewSolutionsReady = true;

                this->isAlgoThreadRunning = false;
              }
            };

            gwoThread.detach();
          } break;
          case AlgorithmType::PSO: {
            std::thread psoThread{
              [this]() {
                // We need to copy the following vectors to this thread, because
                // they somehow get modified as the thread executes.
                auto inputBuildingsCopy = this->inputBuildings;
                auto boundingAreaCopy = this->boundingArea;
                auto flowRatesCopy = this->flowRates;

                PSOResult result = this->psoAlgo.generateSolutions(
                  inputBuildingsCopy,
                  boundingAreaCopy,
                  flowRatesCopy,
                  1.f,
                  this->settings.getIntegerVariable("psoNumIterations").value,
                  this->settings.getIntegerVariable("psoNumParticles").value,
                  this->settings.getFloatVariable("psoW").value,
                  this->settings.getFloatVariable("psoC1").value,
                  this->settings.getFloatVariable("psoC2").value);

                std::cout << "Finished optimizing!\n";

                this->solutionBuffer = result.solutions;
                this->recentRunAvgFitnesses = eastl::vector<double>(
                  result.averageFitnesses);
                this->recentRunBestFitnesses = eastl::vector<double>(
                  result.bestFitnesses);
                this->recentRunWorstFitnesses = eastl::vector<double>(
                  result.worstFitnesses);
                this->recentRunElapsedTime = result.elapsedTime;

                auto time = std::chrono::system_clock::to_time_t(
                  std::chrono::system_clock::now());
                std::stringstream resultsFileRelPath;
                resultsFileRelPath << "pso-results-" << std::to_string(time);
                result.saveToCSV(cx::stdStrToEAStr(resultsFileRelPath.str()));

                // NOTE: There's a weird bug that occurs when we don't subtract
                //       the solution size by one when assigning the value o
                //       the currently selected generation.
                //
                //       I could fix this now, but it's not a priority. Too bad.
                this->currSelectedIter = this->solutionBuffer.size() - 1;
                this->currSelectedIterSolution = 0;
                this->hasSolutionBeenSetup = false;
                this->areNewSolutionsReady = true;

                this->isAlgoThreadRunning = false;
              }
            };

            psoThread.detach();
          } break;
          case AlgorithmType::HC:
            break;
        }
      }
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildAlgorithmResultsWindow()
  {
    ImGui::Begin("GA Results");
    ImGui::BeginChild("GA Results Content");

    ImGui::Text("Solution Fitness: %f",
                (this->currentSolution) ? this->currentSolution->getFitness()
                                        : 0.f);

    double elapsedTime = this->recentRunElapsedTime;
    int32_t hours = elapsedTime / 3600;

    elapsedTime = fmod(elapsedTime, 3600.0);
    int32_t minutes = elapsedTime / 60;

    elapsedTime = fmod(elapsedTime, 60.0);
    int32_t seconds = static_cast<int32_t>(elapsedTime);

    ImGui::Text("Elapsed Time: %02d:%02d:%02d", hours, minutes, seconds);

    // Display combo box for selecting the generation and solution.
    if (ImGui::BeginCombo("Generation",
                          eastl::to_string(currSelectedIter).c_str())) {
      for (int32_t i = 0; i < this->solutions.size(); i++) {
        // Note that i = 0 is the generation with random solutions.
        const bool isItemSelected = (this->currSelectedIter == i);
        if (ImGui::Selectable(eastl::to_string(i).c_str(), isItemSelected)) {
          this->currSelectedIter = i;
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
                          eastl::to_string(currSelectedIterSolution).c_str())) {
      if (this->solutions.size() > 0) {
        for (int32_t i = 0; i < this->solutions[currSelectedIter].size(); i++) {
          // Note that i = 0 is the generation with random solutions.
          const bool isItemSelected = (this->currSelectedIterSolution == i);
          if (ImGui::Selectable(eastl::to_string(i).c_str(), isItemSelected)) {
            this->currSelectedIterSolution = i;
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
        this->currSelectedIter = corex::core::mod(
          this->currSelectedIter - 1,
          this->solutions.size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Next Generation")) {
      if (this->solutions.size() > 0) {
        this->currSelectedIter = corex::core::mod(
          this->currSelectedIter + 1,
          this->solutions.size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::NextColumn();

    if (ImGui::Button("Previous Solution")) {
      if (this->solutions.size() > 0) {
        this->currSelectedIterSolution = corex::core::mod(
          this->currSelectedIterSolution - 1,
          this->solutions[currSelectedIter].size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::SameLine();

    if (ImGui::Button("Next Solution")) {
      if (this->solutions.size() > 0) {
        this->currSelectedIterSolution = corex::core::mod(
          this->currSelectedIterSolution + 1,
          this->solutions[currSelectedIter].size());
        this->hasSolutionBeenSetup = false;
      }
    }

    ImGui::Columns(1);

    if (this->solutions.size() > 0) {
      if (ImGui::Button(!(this->isResultsTimelinePlaying) ? "Play GA Timeline"
                                                          : "Pause GA Timeline")) {
        this->isResultsTimelinePlaying = !(this->isResultsTimelinePlaying);
        this->hasSolutionBeenSetup = false;
      }
    }

    if (ImGui::InputFloat("Time Per Iteration", &this->timePerIteration)) {
      this->timePerIteration = std::max(0.f, this->timePerIteration);
    }

    ImGui::SliderInt("Timeline Playback Speed",
                     &(this->resultsTimelinePlaybackSpeed),
                     1,
                     4);

    ImGui::Separator();

    float avgFitnesses[this->recentRunAvgFitnesses.size()];
    for (int32_t i = 0; i < this->recentRunAvgFitnesses.size(); i++) {
      avgFitnesses[i] = this->recentRunAvgFitnesses[i];
    }

    float bestFitnesses[this->recentRunBestFitnesses.size()];
    for (int32_t i = 0; i < this->recentRunBestFitnesses.size(); i++) {
      bestFitnesses[i] = this->recentRunBestFitnesses[i];
    }

    float worstFitnesses[this->recentRunWorstFitnesses.size()];
    for (int32_t i = 0; i < this->recentRunWorstFitnesses.size(); i++) {
      worstFitnesses[i] = this->recentRunWorstFitnesses[i];
    }

    ImPlot::SetNextPlotLimits(0.0, 1000, 0.0, 500.0);
    if (ImPlot::BeginPlot("Fitness Over Gen. ID", "Generation", "Fitness")) {
      if (this->showResultsAverage) {
        ImPlot::PlotLine("Average Fitness",
                         avgFitnesses,
                         this->recentRunAvgFitnesses.size());
      }

      if (this->showResultsBest) {
        ImPlot::PlotLine("Best Fitness",
                         bestFitnesses,
                         this->recentRunBestFitnesses.size());
      }

      if (this->showResultsWorst) {
        ImPlot::PlotLine("Worst Fitness",
                         worstFitnesses,
                         this->recentRunWorstFitnesses.size());
      }

      ImPlot::EndPlot();
    }

    ImGui::Checkbox("Show Average Fitness", &(this->showResultsAverage));
    ImGui::Checkbox("Show Best Fitness", &(this->showResultsBest));
    ImGui::Checkbox("Show Worst Fitness", &(this->showResultsWorst));

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildDebugSolutionWindow()
  {
    ImGui::Begin("Solution Debugger");
    ImGui::BeginChild("Solution Debugger Content");

    float floatInputBuffer;

    if (this->hasSolutionBeenSetup && !this->isResultsTimelinePlaying) {
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

        floatInputBuffer = this->currentSolution->getBuildingAngle(i);
        if (ImGui::InputFloat(angleText, &floatInputBuffer)) {
          this->currentSolution->setBuildingAngle(i, floatInputBuffer);
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
    if (this->isResultsTimelinePlaying) {
      timeElapsed += timeDelta;

      const float timePerGeneration = this->timePerIteration
                                      / this->resultsTimelinePlaybackSpeed;
      if (corex::core::floatGreEqual(timeElapsed, timePerGeneration)) {
        this->currSelectedIter = corex::core::mod(
          this->currSelectedIter + 1,
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

  void MainScene::clearCurrentlyRenderedSolution()
  {
    for (entt::entity& e : this->buildingEntities) {
      this->registry.destroy(e);
    }

    for (entt::entity& e : this->buildingTextEntities) {
      this->registry.destroy(e);
    }

    if (this->currentAlgorithm == AlgorithmType::GWO) {
      for (entt::entity& e : this->gwoPreyEntities) {
        this->registry.destroy(e);
      }
    }

    this->buildingEntities.clear();
    this->buildingTextEntities.clear();

    if (this->currentAlgorithm == AlgorithmType::GWO) {
      this->gwoPreyEntities.clear();
    }
  }

  void MainScene::drawGWOSolBuildingData(const cx::VecN& data,
                                         const int32_t buildingIdx)
  {
    ImGui::PushItemWidth(ImGui::GetFontSize() * 6);

    for (int32_t i = 0; i < 3; i++) {
      ImGui::SameLine();
      ImGui::Text("%f", data[(buildingIdx * 3) + i]);
    }

    ImGui::PopItemWidth();
  }
}
