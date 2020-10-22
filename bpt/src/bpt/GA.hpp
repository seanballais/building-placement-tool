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
      const float mutationRate,
      const int32_t populationSize,
      const int32_t numGenerations,
      const int32_t tournamentSize);
    double getSolutionFitness(
      const Solution& solution,
      const eastl::vector<InputBuilding>& inputBuildings);
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
    void applySwapping(Solution& solution,
                       const eastl::vector<InputBuilding>& inputBuildings);
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
    eastl::vector<float> recentRunAvgFitnesses;
    eastl::vector<float> recentRunBestFitnesses;
    eastl::vector<float> recentRunWorstFitnesses;
  };
}

#endif
