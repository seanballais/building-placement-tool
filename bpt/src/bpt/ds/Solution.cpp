#include <cassert>
#include <cstdlib>

#include <corex/core/math_functions.hpp>

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution::Solution()
    : genes()
    , numBuildings(0)
    , fitness(0)
    , hasFitnessSet(false) {}

  Solution::Solution(const Solution& other)
    : genes(other.genes)
    , numBuildings(other.numBuildings)
    , fitness(other.fitness)
    , hasFitnessSet(other.hasFitnessSet) {}

  Solution::Solution(int32_t numBuildings)
    : genes(numBuildings * 3, 0.f)
    , numBuildings(numBuildings)
    , fitness(0)
    , hasFitnessSet(false) {}

  void Solution::setBuildingXPos(int32_t buildingIndex, float xPos)
  {
    this->genes[(buildingIndex * 3)] = xPos;
  }

  void Solution::setBuildingYPos(int32_t buildingIndex, float yPos)
  {
    this->genes[(buildingIndex * 3) + 1] = yPos;
  }

  void Solution::setBuildingRotation(int32_t buildingIndex, float rotation)
  {
    this->genes[(buildingIndex * 3) + 2] = rotation;
  }

  void Solution::setFitness(double fitness)
  {
    this->fitness = fitness;
    this->hasFitnessSet = true;
  }

  float Solution::getBuildingXPos(int32_t buildingIndex) const
  {
    return this->genes[(buildingIndex * 3)];
  }

  float Solution::getBuildingYPos(int32_t buildingIndex) const
  {
    return this->genes[(buildingIndex * 3) + 1];
  }

  float Solution::getBuildingRotation(int32_t buildingIndex) const
  {
    return this->genes[(buildingIndex * 3) + 2];
  }

  int32_t Solution::getNumBuildings() const
  {
    return this->numBuildings;
  }

  double Solution::getFitness() const
  {
    assert(this->hasFitnessSet);
    return this->fitness;
  }

  bool Solution::operator==(const Solution& other)
  {
    if (this->genes.size() == other.genes.size()) {
      for (int32_t i = 0; i < this->genes.size(); i++) {
        if (!corex::core::floatEquals(this->genes[i], other.genes[i])) {
          return false;
        }
      }

      return true;
    }

    return false;
  }

  bool Solution::operator!=(const Solution& other)
  {
    return !((*this) == other);
  }
}