#include <bpt/ds/Wolf.hpp>

namespace bpt
{
  double Wolf::getFitness() const
  {
    return this->currSolution.getFitness();
  }

  void Wolf::setFitness(double fitness)
  {
    this->currSolution.setFitness(fitness);
  }

  void Wolf::setCurrAsBest()
  {
    this->bestSolution = this->currSolution;
  }
}

#endif
