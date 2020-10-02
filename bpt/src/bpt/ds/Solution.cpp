#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>

#include <EASTL/array.h>

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

  float Solution::getBuildingXPos(int32_t buildingIndex)
  {
    return this->genes[(buildingIndex * 3)];
  }

  float Solution::getBuildingYPos(int32_t buildingIndex)
  {
    return this->genes[(buildingIndex * 3) + 1];
  }

  float Solution::getBuildingRotation(int32_t buildingIndex)
  {
    return this->genes[(buildingIndex * 3) + 2];
  }

  int32_t Solution::getNumBuildings()
  {
    return this->numBuildings;
  }

  eastl::array<Solution, 2> Solution::crossover(Solution& other)
  {
    // Assume one-point crossover for now.
    std::default_random_engine randGenerator;
    std::uniform_int_distribution<int32_t> geneDistribution{
      0, this->getNumBuildings() - 1
    };

    int32_t pointA = geneDistribution(randGenerator);
    int32_t pointB = 0;
    do {
      pointB = geneDistribution(randGenerator);
    } while (pointB == pointA);

    if (pointB < pointA) {
      std::swap(pointA, pointB);
    }

    Solution childA = *this;
    Solution childB = other;
    for (int32_t currPoint = pointA; currPoint <= pointB; currPoint++) {
      int32_t currPointIdx = currPoint * 3;
      std::swap(childA.genes[currPointIdx], childB.genes[currPointIdx]);
      std::swap(childA.genes[currPointIdx + 1], childB.genes[currPointIdx + 1]);
      std::swap(childA.genes[currPointIdx + 2], childB.genes[currPointIdx + 2]);
    }

    return eastl::array<Solution, 2>{ childA, childB };
  }

  void Solution::mutate()
  {
    // Let's mutate the rotation only for now.
    std::default_random_engine randGenerator;
    std::uniform_int_distribution<int32_t> geneDistribution{
      0, this->getNumBuildings() - 1
    };

    int32_t targetGeneIndex = geneDistribution(randGenerator);

    std::uniform_real_distribution<float> rotationDistribution{ 0.f, 360.f };
    float newRotation = rotationDistribution(randGenerator);

    this->genes[targetGeneIndex + 2] = newRotation;
  }
}