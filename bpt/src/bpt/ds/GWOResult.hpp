#ifndef BPT_DS_GWO_RESULT_HPP
#define BPT_DS_GWO_RESULT_HPP

#include <EASTL/vector.h>

#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct GWOResult : public Result
  {
    GWOResult(const eastl::vector<eastl::vector<Solution>> solutions,
              const eastl::vector<InputBuilding> inputBuildings,
              const eastl::vector<double> bestFitnesses,
              const eastl::vector<double> averageFitnesses,
              const eastl::vector<double> worstFitnesses,
              const double elapsedTime,
              const int32_t numWolves,
              const int32_t numIterations,
              const float alphaDecayRate);

    const int32_t numWolves;
    const int32_t numIterations;
    const float alphaDecayRate;

  private:
    void customSaveToCSV(std::ofstream& resultsFile) override;
  };
}

#endif
