#include <algorithm>
#include <cstdlib>
#include <random>

#include <EASTL/array.h>

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution::Solution(const Solution& other)
    : genes(other.genes)
    , numBuildings(other.numBuildings) {}

  Solution::Solution(int32_t numBuildings)
    : genes(numBuildings * 3, 0.f)
    , numBuildings(numBuildings) {}

  void Solution::setBuildingXPos(int32_t buildingIndex, float xPos)
  {
    this->genes[buildingIndex] = xPos;
  }

  void Solution::setBuildingYPos(int32_t buildingIndex, float yPos)
  {
    this->genes[buildingIndex + 1] = yPos;
  }

  void Solution::setBuildingRotation(int32_t buildingIndex, float rotation)
  {
    this->genes[buildingIndex + 2] = rotation;
  }

  float Solution::getBuildingXPos(int32_t buildingIndex)
  {
    return this->genes[buildingIndex];
  }

  float Solution::getBuildingYPos(int32_t buildingIndex)
  {
    return this->genes[buildingIndex + 1];
  }

  float Solution::getBuildingRotation(int32_t buildingIndex)
  {
    return this->genes[buildingIndex + 2];
  }

  int32_t getNumBuildings()
  {
    return this->numBuildings;
  }

  eastl::array<Solution, 2> crossover(Solution& other)
  {
    // Assume one-point crossover for now.
    std::default_random_engine randGenerator;
    std::uniform_int_distribution<int32_t> chromosomeDistribution{
      0, this->getNumBuildings() - 1
    };

    int32_t pointA = chromosomeDistribution(randGenerator);
    int32_t pointB = 0;
    do {
      pointB = chromosomeDistribution(randGenerator);
    } while (pointBs == pointA);

    if (pointB < pointA) {
      std::swap(pointA, pointB);
    }

    Solution childA = *this;
    Solution childB = other;
    for (int32_t currPoint = pointA; currPoint <= pointB; currPoint++) {
      std::swap(childA.genes[currPoint], childB.genes[currPoint]);
      std::swap(childA.genes[currPoint + 1], childB.genes[currPoint + 1]);
      std::swap(childA.genes[currPoint + 2], childB.genes[currPoint + 2]);
    }

    return eastl::array<Solution, 2>{ childA, childB };
  }
}