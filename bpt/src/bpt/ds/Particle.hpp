#ifndef BPT_DS_PSO_SOLUTION_HPP
#define BPT_DS_PSO_SOLUTION_HPP

#include <corex/core/ds/VecN.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct Particle
  {
    Solution pBest;
    Solution currSol;
    cx::VecN velocity;
  };
}

#endif
