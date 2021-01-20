#ifndef BPT_GD_HPP
#define BPT_GD_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  class GD
  {
  public:
    Solution generateSolution(
      Solution initialSolution,
      const eastl::vector<InputBuilding> &inputBuildings,
      const corex::core::NPolygon &boundingArea,
      const eastl::vector<eastl::vector<float>> &flowRates,
      const eastl::vector<corex::core::NPolygon> &floodProneAreas,
      const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const double timeLimit,
      const float buildingDistanceWeight,
      const float minExpectedPenalty,
      const float minDecayRate,
      const float maxDecayRate,
      const float decayMultiplier,
      const double minWaterLevelIncrease,
      const double maxWaterLevelIncrease,
      const bool keepInfeasibleSolutions);
  private:
    void perturbSolution(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight,
      const bool keepInfeasibleSolutions);
    void applyBuddyBuddyMove(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const int32_t staticBuildingIndex = -1,
      const int32_t dynamicBuildingIndex = -1,
      const bool& keepInfeasibleSolutions = true);
    void applyShakingMove(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    void applyJiggleMove(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
  };
}

#endif
