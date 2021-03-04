#ifndef BPT_OPERATORS_HPP
#define BPT_OPERATORS_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  void applyBuddyBuddyOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const int32_t staticBuildingIndex = -1,
    const int32_t dynamicBuildingIndex = -1,
    const bool& keepInfeasibleSolutions = true);
  void applyShakingOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions);
  void applyJiggleOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions);
}

#endif
