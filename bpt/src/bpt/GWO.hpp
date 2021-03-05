#ifndef BPT_GWO_HPP
#define BPT_GWO_HPP

#include <cstdlib>

#include <EASTL/vector.h>

#include <corex/core/Timer.hpp>
#include <corex/core/ds/NPolygon.hpp>

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Result.hpp>

namespace bpt
{
  class GWO
  {
  public:
    GWO();
    Result generateSolutions(
      const eastl::vector<InputBuilding> &inputBuildings,
      const corex::core::NPolygon &boundingArea,
      const eastl::vector<eastl::vector<float>> &flowRates,
      const eastl::vector<corex::core::NPolygon> &floodProneAreas,
      const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
      const int32_t numWolves,
      const int32_t numIterations,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight,
      const CrossoverType crossoverType,
      const bool& keepInfeasibleSolutions);
  private:
    void computeWolfValues(
      eastl::vector<Solution>& wolves,
      eastl::vector<double>& wolfMutationRates,
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight);
    void updateWolves(
      eastl::vector<Solution>& wolves,
      const CrossoverType crossoverType,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    void mutateWolves(
      eastl::vector<Solution>& wolves,
      eastl::vector<double>& wolfMutationRates,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    eastl::vector<Solution> crossoverSolutions(
      const CrossoverType& type,
      const Solution& solutionA,
      const Solution& solutionB,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    void mutateSolution(Solution& solution,
                        const corex::core::NPolygon& boundingArea,
                        const eastl::vector<InputBuilding>& inputBuildings,
                        const bool& keepInfeasibleSolutions);

    cx::Timer runTimer;
  };
}

#endif
