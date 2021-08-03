#ifndef BPT_MAIN_SCENE_HPP
#define BPT_MAIN_SCENE_HPP

#include <atomic>

#include <EASTL/vector.h>
#include <entt/entt.hpp>
#include <nlohmann/json.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/Settings.hpp>
#include <corex/core/ds/Circle.hpp>
#include <corex/core/ds/LineSegments.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/events/KeyboardEvent.hpp>
#include <corex/core/events/MouseButtonEvent.hpp>
#include <corex/core/events/MouseMovementEvent.hpp>
#include <corex/core/events/MouseScrollEvent.hpp>
#include <corex/core/events/sys_events.hpp>

#include <bpt/Context.hpp>
#include <bpt/GA.hpp>
#include <bpt/GWO.hpp>
#include <bpt/HC.hpp>
#include <bpt/PSO.hpp>
#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/GASettings.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/LSSettings.hpp>
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
    void buildInputBuildingsWindow();
    void buildFlowRateWindow();
    void buildAlgorithmControlsWindow();
    void buildAlgorithmResultsWindow();
    void buildDebugSolutionWindow();
    void handleGATimelinePlayback(float timeDelta);
    void handleWindowEvents(const corex::core::WindowEvent& e);
    void handleKeyboardEvents(const corex::core::KeyboardEvent& e);
    void handleMouseButtonEvents(const corex::core::MouseButtonEvent& e);
    void handleMouseMovementEvents(const corex::core::MouseMovementEvent& e);
    void handleMouseScrollEvents(const corex::core::MouseScrollEvent& e);
    void clearCurrentlyRenderedSolution();
    void drawGWOSolBuildingData(const cx::VecN &data,
                                const int32_t buildingIdx);

    Context currentContext;
    GA geneticAlgo;
    GWO gwoAlgo;
    PSO psoAlgo;
    HC hillClimbingAlgo;
    AlgorithmType currentAlgorithm;
    GASettings gaSettings;
    LSSettings lsSettings;
    cx::Settings settings;
    Solution* currentSolution;
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<eastl::vector<Solution>> solutionBuffer;
    std::atomic<bool> isAlgoThreadRunning;
    bool hasSolutionBeenSetup;
    bool doesInputDataExist;
    bool doesInputBoundingAreaFieldExist;
    bool doesInputBuildingsExist;
    bool doesGASettingsFieldExist;
    bool doFloodProneAreasExist;
    bool doLandslideProneAreasExist;
    bool isCloseAreaTriggerEnabled;
    bool showResultsAverage;
    bool showResultsBest;
    bool showResultsWorst;
    bool needUpdateBuildingRenderMode;
    bool isResultsTimelinePlaying;
    float cameraMovementSpeed;
    float timeDelta;
    float timePerIteration;
    int32_t currSelectedIter;
    int32_t currSelectedIterSolution;
    int32_t resultsTimelinePlaybackSpeed;
    corex::core::Circle closeAreaTriggerCircle;
    corex::core::LineSegments wipBoundingArea;
    corex::core::LineSegments wipHazardArea;
    corex::core::NPolygon boundingArea;
    eastl::vector<corex::core::Polygon<3>> boundingAreaTriangles;
    eastl::vector<corex::core::NPolygon> floodProneAreas;
    eastl::vector<corex::core::NPolygon> landslideProneAreas;
    entt::entity wipBoundingAreaEntity;
    entt::entity wipHazardAreaEntity;
    entt::entity boundingAreaEntity;
    entt::entity closeAreaTriggerEntity;
    eastl::vector<entt::entity> boundingAreaTriangleEntities;
    eastl::vector<entt::entity> floodProneAreaEntities;
    eastl::vector<entt::entity> landslideProneAreaEntities;
    eastl::vector<InputBuilding> inputBuildings;
    eastl::vector<entt::entity> buildingEntities;
    eastl::vector<entt::entity> gwoPreyEntities;
    eastl::vector<eastl::vector<float>> flowRates;
    eastl::vector<entt::entity> buildingTextEntities;
    eastl::vector<entt::entity> floodProneAreaTextEntities;
    eastl::vector<entt::entity> landslideProneAreaTextEntities;
    eastl::vector<double> recentRunAvgFitnesses;
    eastl::vector<double> recentRunBestFitnesses;
    eastl::vector<double> recentRunWorstFitnesses;
    double recentRunElapsedTime;
    nlohmann::json inputData;
    bool areNewSolutionsReady;
  };
}

#endif
