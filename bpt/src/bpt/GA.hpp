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
    Solution generateSolution(eastl::vector<InputBuilding>& inputBuildings,
                              corex::core::NPolygon& boundingArea,
                              float mutationRate,
                              int32_t populationSize,
                              int32_t numGenerations);
    double getSolutionFitness(Solution& solution);
    eastl::vector<float> getRecentRunAverageFitnesses();
    eastl::vector<float> getRecentRunBestFitnesses();
    eastl::vector<float> getRecentRunWorstFitnesses();
  private:
    Solution
    generateRandomSolution(eastl::vector<InputBuilding>& inputBuildings,
                           corex::core::NPolygon& boundingArea);
    eastl::vector<float> recentRunAvgFitnesses;
    eastl::vector<float> recentRunBestFitnesses;
    eastl::vector<float> recentRunWorstFitnesses;
  };
}

#endif
