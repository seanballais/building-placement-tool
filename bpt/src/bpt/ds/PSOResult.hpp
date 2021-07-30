#ifndef BPT_DS_PSO_RESULT_HPP
#define BPT_DS_PSO_RESULT_HPP

#include <EASTL/vector.h>

#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct PSOResult : public Result
  {
    PSOResult(const eastl::vector<eastl::vector<Solution>> solutions,
              const eastl::vector<InputBuilding> inputBuildings,
              const eastl::vector<double> bestFitnesses,
              const eastl::vector<double> averageFitnesses,
              const eastl::vector<double> worstFitnesses,
              const double elapsedTime,
              const int32_t numParticles,
              const int32_t numIterations,
              const float w,
              const float c1,
              const float c2);

    const int32_t numParticles;
    const int32_t numIterations;
    const float w;  // Inertial weight.
    const float c1; // Selfishness factor.
    const float c2; // Social factor.

  private:
    void customSaveToCSV(std::ofstream& resultsFile) override;
  };
}

#endif
