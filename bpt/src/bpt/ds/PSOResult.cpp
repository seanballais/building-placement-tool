#include <fstream>

#include <EASTL/vector.h>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/PSOResult.hpp>
#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  PSOResult::PSOResult(const eastl::vector<eastl::vector<Solution>> solutions,
                       const eastl::vector<InputBuilding> inputBuildings,
                       const eastl::vector<double> bestFitnesses,
                       const eastl::vector<double> averageFitnesses,
                       const eastl::vector<double> worstFitnesses,
                       const double elapsedTime,
                       const int32_t numParticles,
                       const int32_t numIterations,
                       const float w,
                       const float c1,
                       const float c2)
    : Result(solutions,
             inputBuildings,
             bestFitnesses,
             averageFitnesses,
             worstFitnesses,
             elapsedTime)
    , numParticles(numParticles)
    , numIterations(numIterations)
    , w(w)
    , c1(c1)
    , c2(c2) {}

  void PSOResult::customSaveToCSV(std::ofstream& resultsFile)
  {
    resultsFile << "\n";
    resultsFile << "PSO Parameters\n"
                << "No. of Particles:," << this->numParticles
                << "\n"
                << "No. of Iterations:," << this->numIterations
                << "\n"
                << "Inertial Weight:," << this->w
                << "\n"
                << "Selfishness Factor:," << this->c1
                << "\n"
                << "Social Factor:," << this->c2
                << "\n";
  }
}
