#ifndef BPT_DS_RESULT_HPP
#define BPT_DS_RESULT_HPP

#include <EASTL/vector.h>

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct Result
  {
    const eastl::vector<eastl::vector<Solution>> solutions;
    const eastl::vector<double> bestFitnesses;
    const eastl::vector<double> averageFitnesses;
    const eastl::vector<double> worstFitnesses;
    double elapsedTime;
  };
}

#endif
