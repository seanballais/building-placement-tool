#ifndef BPT_DS_GA_RESULT_HPP
#define BPT_DS_GA_RESULT_HPP

#include <EASTL/vector.h>

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct GAResult : public Result
  {
    GAResult(const eastl::vector<eastl::vector<Solution>> solutions,
             const eastl::vector<InputBuilding> inputBuildings,
             const eastl::vector<double> bestFitnesses,
             const eastl::vector<double> averageFitnesses,
             const eastl::vector<double> worstFitnesses,
             const double elapsedTime,
             const float mutationRate,
             const int32_t populationSize,
             const int32_t numGenerations,
             const SelectionType selectionType,
             const int32_t tournamentSize,
             const CrossoverType crossoverType);

    const float mutationRate;
    const int32_t populationSize;
    const int32_t numGenerations;
    const SelectionType selectionType;
    const int32_t tournamentSize;
    const CrossoverType crossoverType;

  private:
    void customSaveToCSV(std::ofstream& resultsFile) override;
  };
}

#endif
