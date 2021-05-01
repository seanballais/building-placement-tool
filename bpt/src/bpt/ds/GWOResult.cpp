#include <fstream>

#include <EASTL/vector.h>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/GWOResult.hpp>
#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  GWOResult::GWOResult(const eastl::vector<eastl::vector<Solution>> solutions,
                       const eastl::vector<InputBuilding> inputBuildings,
                       const eastl::vector<double> bestFitnesses,
                       const eastl::vector<double> averageFitnesses,
                       const eastl::vector<double> worstFitnesses,
                       const double elapsedTime,
                       const int32_t numWolves,
                       const int32_t numIterations,
                       const float alphaDecayRate)
    : Result(solutions,
             inputBuildings,
             bestFitnesses,
             averageFitnesses,
             worstFitnesses,
             elapsedTime)
    , numWolves(numWolves)
    , numIterations(numIterations)
    , alphaDecayRate(alphaDecayRate) {}

  void GWOResult::customSaveToCSV(std::ofstream& resultsFile)
  {
    resultsFile << "\n";
    resultsFile << "GWO Parameters\n"
                << "No. of Wolves:," << this->numWolves
                << "\n"
                << "No. of Iterations:," << this->numIterations
                << "\n"
                << "Alpha Decay Rate:," << this->alphaDecayRate
                << "\n";
  }
}
