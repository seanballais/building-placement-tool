#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <corex/core/utils.hpp>

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution::Solution()
    : genes()
    , numBuildings(0) {}

  Solution::Solution(const Solution& other)
    : genes(other.genes)
    , numBuildings(other.numBuildings) {}

  Solution::Solution(int32_t numBuildings)
    : genes(numBuildings * 3, 0.f)
    , numBuildings(numBuildings) {}

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
}