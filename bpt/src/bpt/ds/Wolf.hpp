#ifndef BPT_DS_WOLF_HPP
#define BPT_DS_WOLF_HPP

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct Wolf
  {
    Solution currSolution;
    Solution bestSolution;

    [[nodiscard]] double getFitness() const;
    void setFitness(double fitness);
    void setCurrAsBest();
  };
}

#endif
