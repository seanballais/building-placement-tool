#include <EASTL/vector.h

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct Result
  {
    const eastl::vector<eastl::vector<Solution>> solutions;
    const eastl::vector<float> averageFitnesses;
    const eastl::vector<float> bestFitnesses;
    const eastl::vector<float> worstFitnesses;
    double elapsedTime;
  };
}
