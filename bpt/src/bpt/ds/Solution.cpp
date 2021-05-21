#include <cassert>
#include <cstdlib>
#include <iostream>

#include <corex/core/math_functions.hpp>

#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution::Solution()
    : genes()
    , genesAssignmentStatus()
    , numBuildings(0)
    , fitness(0)
    , hasFitnessSet(false) {}

  Solution::Solution(const Solution& other)
    : genes(other.genes)
    , genesAssignmentStatus(other.genesAssignmentStatus)
    , numBuildings(other.numBuildings)
    , fitness(other.fitness)
    , hasFitnessSet(other.hasFitnessSet) {}

  Solution::Solution(int32_t numBuildings)
    : genes(numBuildings * 3, 0.f)
    , genesAssignmentStatus(numBuildings, false)
    , numBuildings(numBuildings)
    , fitness(0)
    , hasFitnessSet(false) {}

  void Solution::setBuildingXPos(int32_t buildingIndex, float xPos)
  {
    this->genes[(buildingIndex * 3)] = xPos;
    this->genesAssignmentStatus[buildingIndex] = true;
  }

  void Solution::setBuildingYPos(int32_t buildingIndex, float yPos)
  {
    this->genes[(buildingIndex * 3) + 1] = yPos;
    this->genesAssignmentStatus[buildingIndex] = true;
  }

  void Solution::setBuildingAngle(int32_t buildingIndex, float rotation)
  {
    this->genes[(buildingIndex * 3) + 2] = rotation;
    this->genesAssignmentStatus[buildingIndex] = true;
  }

  void Solution::moveBuildingXPos(int32_t buildingIndex,
                                  float xDelta,
                                  float minX,
                                  float maxX)
  {
    this->setBuildingXPos(
      buildingIndex,
      cx::clamp(this->getBuildingXPos(buildingIndex) + xDelta, minX, maxX));
  }

  void Solution::moveBuildingYPos(int32_t buildingIndex,
                                  float yDelta,
                                  float minY,
                                  float maxY)
  {
    this->setBuildingYPos(
      buildingIndex,
      cx::clamp(this->getBuildingYPos(buildingIndex) + yDelta, minY, maxY));
  }

  void Solution::moveBuildingXPos(int32_t buildingIndex, float xDelta)
  {
    this->setBuildingXPos(buildingIndex,
                          this->getBuildingXPos(buildingIndex) + xDelta);
  }

  void Solution::moveBuildingYPos(int32_t buildingIndex, float yDelta)
  {
    this->setBuildingYPos(buildingIndex,
                          this->getBuildingYPos(buildingIndex) + yDelta);
  }

  void Solution::setFitness(double fitness)
  {
    this->fitness = fitness;
    this->hasFitnessSet = true;
  }

  void Solution::validateBuildingData(int32_t buildingIndex)
  {
    this->genesAssignmentStatus[buildingIndex] = true;
  }

  void Solution::invalidateBuildingData(int32_t buildingIndex)
  {
    this->genesAssignmentStatus[buildingIndex] = false;
  }

  bool Solution::isBuildingDataUsable(int32_t buildingIndex) const
  {
    return this->genesAssignmentStatus[buildingIndex];
  }

  float Solution::getBuildingXPos(int32_t buildingIndex) const
  {
    assert(this->isBuildingDataUsable(buildingIndex));
    return this->genes[(buildingIndex * 3)];
  }

  float Solution::getBuildingYPos(int32_t buildingIndex) const
  {
    assert(this->isBuildingDataUsable(buildingIndex));
    return this->genes[(buildingIndex * 3) + 1];
  }

  float Solution::getBuildingAngle(int32_t buildingIndex) const
  {
    assert(this->isBuildingDataUsable(buildingIndex));
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