#ifndef BPT_MAIN_SCENE_HPP
#define BPT_MAIN_SCENE_HPP

#include <atomic>

#include <EASTL/vector.h>
#include <entt/entt.hpp>
#include <nlohmann/json.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/ds/Circle.hpp>
#include <corex/core/ds/LineSegments.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/MouseScrollEvent.hpp>
#include <corex/core/events/sys_events.hpp>

#include <bpt/Context.hpp>
#include <bpt/GA.hpp>
#include <bpt/ds/GASettings.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  class MainScene : public corex::core::Scene
  {
  public:
    MainScene(entt::registry& registry,
              entt::dispatcher& eventDispatcher,
              corex::core::AssetManager& assetManager,
              corex::core::Camera& camera);

    void init() override;
    void update(float timeDelta) override;
    void dispose() override;

  private:
    void buildConstructBoundingAreaWindow();
    void buildWarningWindow();
    void buildInputBuildingsWindow();
    void buildGAControlsWindow();
    void buildGAResultsWindow();
    void handleWindowEvents(const corex::core::WindowEvent& e);
    void handleKeyboardEvents(const corex::core::KeyboardEvent& e);
    void handleMouseButtonEvents(const corex::core::MouseButtonEvent& e);
    void handleMouseMovementEvents(const corex::core::MouseMovementEvent& e);
    void handleMouseScrollEvents(const corex::core::MouseScrollEvent& e);
    void saveResultsToCSVFile();

    Context currentContext;
    GA geneticAlgo;
    GASettings gaSettings;
    Solution currentSolution;
    std::atomic<bool> isGAThreadRunning;
    bool doesInputDataExist;
    bool doesInputBoundingAreaFieldExist;
    bool doesInputBuildingsExist;
    bool doesGASettingsFieldExist;
    bool isCloseAreaTriggerEnabled;
    bool showGAResultsAverage;
    bool showGAResultsBest;
    bool showGAResultsWorst;
    bool hasSetupCurrentSolution;
    float cameraMovementSpeed;
    float timeDelta;
    corex::core::Circle closeAreaTriggerCircle;
    corex::core::LineSegments wipBoundingArea;
    corex::core::NPolygon boundingArea;
    entt::entity wipBoundingAreaEntity;
    entt::entity boundingAreaEntity;
    entt::entity closeAreaTriggerEntity;
    eastl::vector<InputBuilding> inputBuildings;
    eastl::vector<entt::entity> buildingEntities;
    eastl::vector<float> recentGARunAvgFitnesses;
    eastl::vector<float> recentGARunBestFitnesses;
    eastl::vector<float> recentGARunWorstFitnesses;
    nlohmann::json inputData;
  };
}

#endif
