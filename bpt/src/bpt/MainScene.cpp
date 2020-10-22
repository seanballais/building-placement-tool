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
#include <imgui_impls/imgui_impl_opengl3.h>
#include <imgui_impls/imgui_impl_sdl.h>
#include <implot/implot.h>
#include <nlohmann/json.hpp>

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
#include <bpt/MainScene.hpp>
#include <bpt/ds/Solution.hpp>
#include <bpt/ds/InputBuilding.hpp>

namespace bpt
{
  MainScene::MainScene(entt::registry& registry,
                       entt::dispatcher& eventDispatcher,
                       corex::core::AssetManager& assetManager,
                       corex::core::Camera& camera)
    : currentContext(Context::NO_ACTION)
    , geneticAlgo()
    , gaSettings({ 0.25f, 25, 1000, 4 })
    , currentSolution()
    , isGAThreadRunning(false)
    , doesInputDataExist(false)
    , doesInputBoundingAreaFieldExist(false)
    , doesInputBuildingsExist(false)
    , doesGASettingsFieldExist(false)
    , isCloseAreaTriggerEnabled(false)
    , showGAResultsAverage(true)
    , showGAResultsBest(true)
    , showGAResultsWorst(true)
    , hasSetupCurrentSolution(false)
    , closeAreaTriggerCircle(corex::core::Circle{
        corex::core::Point{ 0.f, 0.f }, 0.0f
      })
    , cameraMovementSpeed(15.f)
    , timeDelta(0.f)
    , wipBoundingArea()
    , boundingArea()
    , wipBoundingAreaEntity(entt::null)
    , boundingAreaEntity(entt::null)
    , closeAreaTriggerEntity(entt::null)
    , inputBuildings()
    , buildingEntities()
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
              // Sigh. Shoud have used a JSON object here.
              this->boundingArea.vertices.push_back(corex::core::Point{
                vertex[0].get<float>(), vertex[1].get<float>()
              });
            }
            this->doesInputBoundingAreaFieldExist = true;
          }

          if (this->inputData.contains("inputBuildings")) {
            for (auto& building : this->inputData["inputBuildings"]) {
              // Sigh. Shoud have used a JSON object here.
              this->inputBuildings.push_back(InputBuilding{
                building[0].get<float>(), building[1].get<float>()
              });
            }
            this->doesInputBuildingsExist = true;
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

            this->doesGASettingsFieldExist = true;
          }
        }
      }
    }
  }

  void MainScene::update(float timeDelta)
  {
    this->timeDelta = timeDelta;

    if (!this->hasSetupCurrentSolution && !this->isGAThreadRunning) {
      for (int32_t i = 0; i < this->currentSolution.getNumBuildings(); i++) {
        entt::entity e = this->registry.create();
        this->registry.emplace<corex::core::Position>(
          e,
          this->currentSolution.getBuildingXPos(i),
          this->currentSolution.getBuildingYPos(i),
          0.f,
          static_cast<int8_t>(0));
        this->registry.emplace<corex::core::Renderable>(
          e,
          corex::core::RenderableType::PRIMITIVE_RECTANGLE);
        this->registry.emplace<corex::core::RenderRectangle>(
          e,
          this->currentSolution.getBuildingXPos(i),
          this->currentSolution.getBuildingYPos(i),
          this->inputBuildings[i].width,
          this->inputBuildings[i].length,
          this->currentSolution.getBuildingRotation(i),
          SDL_Color{ 0, 102, 51, 255 },
          true);

        this->buildingEntities.push_back(e);

        std::cout << "Building #" << i << std::endl;
        std::cout << "-- x: " << this->currentSolution.getBuildingXPos(i) << std::endl;
        std::cout << "-- y: " << this->currentSolution.getBuildingYPos(i) << std::endl;
        std::cout << "-- Rotation: " << this->currentSolution.getBuildingRotation(i)
                  << std::endl;
      }

      std::cout << "Fitness: "
                << this->geneticAlgo.getSolutionFitness(this->currentSolution,
                                                        this->inputBuildings)
                << std::endl;

      this->hasSetupCurrentSolution = true;
    }

    switch (this->currentContext) {
      case Context::NO_ACTION: {
        if (!this->registry.valid(this->boundingAreaEntity)) {
          this->boundingAreaEntity = this->registry.create();
          this->registry.emplace<corex::core::Position>(
            this->boundingAreaEntity,
            0.f,
            0.f,
            1.f,
            static_cast<int8_t>(1));
          this->registry.emplace<corex::core::Renderable>(
            this->boundingAreaEntity,
            corex::core::RenderableType::PRIMITIVE_POLYGON);
          this->registry.emplace<corex::core::RenderPolygon>(
            this->boundingAreaEntity,
            eastl::vector<corex::core::Point>{},
            SDL_Color{ 64, 64, 64, 255},
            false);
        } else {
          this->registry.patch<corex::core::RenderPolygon>(
            this->boundingAreaEntity,
            [this](corex::core::RenderPolygon& poly) {
              poly.vertices = this->boundingArea.vertices;
            }
          );
        }

        if (this->registry.valid(this->wipBoundingAreaEntity)) {
          this->registry.destroy(this->wipBoundingAreaEntity);
        }

        if (this->registry.valid(this->closeAreaTriggerEntity)) {
          this->registry.destroy(this->closeAreaTriggerEntity);
        }
      } break;
      case Context::DRAW_AREA_BOUND: {
        if (this->isCloseAreaTriggerEnabled) {
          int32_t lastElementIndex = this->wipBoundingArea.vertices.size() - 1;
          float dist = corex::core::distance2D(
            this->wipBoundingArea.vertices[0],
            this->wipBoundingArea.vertices[lastElementIndex]);
          if (corex::core::floatLessEqual(dist, 7.f)) {
            this->closeAreaTriggerCircle.radius = 5.f;
          } else {
            this->closeAreaTriggerCircle.radius = 0.f;
          }
        }

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
          this->registry.destroy(this->boundingAreaEntity);
        }
      } break;
    }

    this->buildConstructBoundingAreaWindow();
    this->buildWarningWindow();
    this->buildInputBuildingsWindow();
    this->buildGAControlsWindow();
    this->buildGAResultsWindow();
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
      for (InputBuilding& building : this->inputBuildings) {
        this->inputData["inputBuildings"].push_back(
          nlohmann::json::array({ building.length, building.width })
        );
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

  void MainScene::buildWarningWindow()
  {
    if (!this->doesInputDataExist
        || !this->doesInputBoundingAreaFieldExist
        || !this->doesInputBuildingsExist
        || !this->doesGASettingsFieldExist) {
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
    }
    ImGui::Separator();

    ImGui::BeginChild("Input Buildings List");

    for (int32_t i = 0; i < this->inputBuildings.size(); i++) {
      ImGui::Separator();

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
    }

    ImGui::EndChild();
    ImGui::End();
  }

  void MainScene::buildGAControlsWindow()
  {
    ImGui::Begin("GA Controls");

    // TODO: Save ga setting to a file.
    ImGui::InputFloat("Mutation Rate", &(this->gaSettings.mutationRate));
    ImGui::InputInt("Population Size", &(this->gaSettings.populationSize));
    ImGui::InputInt("No. of Generations", &(this->gaSettings.numGenerations));
    ImGui::InputInt("Tournament Size", &(this->gaSettings.tournamentSize));

    if (this->isGAThreadRunning) {
      ImGui::Text("Running GA...");
    } else {
      if (ImGui::Button("Generate Solution")) {
        this->isGAThreadRunning = true;
        this->hasSetupCurrentSolution = false;

        for (entt::entity& e : this->buildingEntities) {
          this->registry.destroy(e);
        }

        this->buildingEntities.clear();

        std::thread gaThread{
          [this]() {
            // We need to copy the input buildings and bounding area vectors to
            // this thread,because they somehow get modified as the thread
            // executes.
            auto inputBuildingsCopy = this->inputBuildings;
            auto boundingAreaCopy = this->boundingArea;

            this->currentSolution = this->geneticAlgo.generateSolution(
              inputBuildingsCopy,
              boundingAreaCopy,
              this->gaSettings.mutationRate,
              this->gaSettings.populationSize,
              this->gaSettings.numGenerations,
              this->gaSettings.tournamentSize
            );

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

    ImGui::End();
  }

  void MainScene::buildGAResultsWindow()
  {
    ImGui::Begin("GA Results");
    ImGui::BeginChild("GA Results Content");

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
      }
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
          if (this->isCloseAreaTriggerEnabled) {
            int32_t lastElementIndex = this->wipBoundingArea
                                            .vertices.size() - 1;
            float dist = corex::core::distance2D(
              this->wipBoundingArea.vertices[0],
              this->wipBoundingArea.vertices[lastElementIndex]);
            if (corex::core::floatLessEqual(dist, 7.f)) {
              for (int32_t i = 0;
                   i < this->wipBoundingArea.vertices.size() - 1;
                   i++) {
                // Note: The last elements contains the position of the mouse.
                //       We shouldn't add it, otherwise we'd get another edge
                //       to the bounding area that we don't want.
                this->boundingArea.vertices.push_back(
                  this->wipBoundingArea.vertices[i]);
              }

              this->currentContext = Context::NO_ACTION;
              this->isCloseAreaTriggerEnabled = false;
              this->closeAreaTriggerCircle.position = corex::core
                                                           ::Point{ 0.f, 0.f };
              this->wipBoundingArea.vertices.clear();
              break;
            }
          }

          // Once the number of vertices the WIP bounding area has becomes two,
          // the Point instance we're pushing will hold the current position
          // of the mouse cursor, while we're drawing the bounding area.
          this->wipBoundingArea.vertices.push_back(
            corex::core::screenToWorldCoordinates(
              corex::core::Point{
                static_cast<float>(e.x),
                static_cast<float>(e.y)
              },
              this->camera
            )
          );

          if (this->wipBoundingArea.vertices.size() == 1) {
            this->wipBoundingArea.vertices.push_back(
              corex::core::screenToWorldCoordinates(
                corex::core::Point{
                  static_cast<float>(e.x),
                  static_cast<float>(e.y)
                },
                this->camera
              )
            );
          }

          if (this->wipBoundingArea.vertices.size() == 4) {
            this->closeAreaTriggerCircle.position = this->wipBoundingArea
                                                         .vertices[0];
            this->isCloseAreaTriggerEnabled = true;
          }
        } else if (
            e.buttonType == corex::core::MouseButtonType::MOUSE_BUTTON_RIGHT
            && e.buttonState == corex::core
                                     ::MouseButtonState::MOUSE_BUTTON_DOWN
            && e.numRepeats == 0) {
          this->currentContext = Context::NO_ACTION;
          this->isCloseAreaTriggerEnabled = false;
          this->closeAreaTriggerCircle.position = corex::core
                                                       ::Point{ 0.f, 0.f };
          this->wipBoundingArea.vertices.clear();
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
          this->wipBoundingArea
               .vertices[lastElementIndex] = corex::core
                                                  ::screenToWorldCoordinates(
            corex::core::Point{
              static_cast<float>(e.x),
              static_cast<float>(e.y)
            },
            this->camera
          );
        }

        break;
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
                << "\n";

    resultsFile.close();
  }
}
