#ifndef BPT_GA_HPP
#define BPT_GA_HPP

#include <cstdlib>

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  class GA
  {
  public:
    GA();
    Solution generateSolution(
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<eastl::vector<float>>& flowRates,
      eastl::vector<corex::core::NPolygon>& floodProneAreas,
      eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float mutationRate,
      const int32_t populationSize,
      const int32_t numGenerations,
      const int32_t tournamentSize,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight,
      const bool isLocalSearchEnabled);
    double getSolutionFitness(
      const Solution& solution,
      const eastl::vector<InputBuilding>& inputBuildings,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight);
    int32_t getCurrentRunGenerationNumber();
    eastl::vector<float> getRecentRunAverageFitnesses();
    eastl::vector<float> getRecentRunBestFitnesses();
    eastl::vector<float> getRecentRunWorstFitnesses();
  private:
    Solution
    generateRandomSolution(const eastl::vector<InputBuilding>& inputBuildings,
                           const corex::core::NPolygon& boundingArea);
    eastl::array<Solution, 2>
    crossoverSolutions(const Solution& solutionA,
                       const Solution& solutionB,
                       const corex::core::NPolygon& boundingArea,
                       const eastl::vector<InputBuilding>& inputBuildings);
    void mutateSolution(Solution& solution,
                        const corex::core::NPolygon& boundingArea,
                        const eastl::vector<InputBuilding>& inputBuildings);
    void applyLocalSearch1(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const eastl::vector<eastl::vector<float>>& flowRates);
    bool
    isSolutionFeasible(const Solution& solution,
                       const corex::core::NPolygon& boundingArea,
                       const eastl::vector<InputBuilding>& inputBuildings);
    bool doesSolutionHaveNoBuildingsOverlapping(
      const Solution& solution,
      const eastl::vector<InputBuilding>& inputBuildings);
    bool areSolutionBuildingsWithinBounds(
      const Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings);
    int32_t currRunGenerationNumber;
    eastl::vector<float> recentRunAvgFitnesses;
    eastl::vector<float> recentRunBestFitnesses;
    eastl::vector<float> recentRunWorstFitnesses;
  };
}

#endif
