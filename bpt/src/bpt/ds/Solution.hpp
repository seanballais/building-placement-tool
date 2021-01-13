#ifndef BPT_DS_SOLUTION_HPP
#define BPT_DS_SOLUTION_HPP

#include <cstdlib>

#include <EASTL/vector.h>

namespace bpt
{
  class Solution
  {
    // Solution representation:
    //   [ xPos of building 0, yPos of building 0, rotation of building 0, ... ]
  public:
    Solution();
    Solution(const Solution& other);
    Solution(int32_t numBuildings);

    void setBuildingXPos(int32_t buildingIndex, float xPos);
    void setBuildingYPos(int32_t buildingIndex, float yPos);
    void setBuildingAngle(int32_t buildingIndex, float rotation);
    void setFitness(double fitness);
    void validateBuildingData(int32_t buildingIndex);
    void invalidateBuildingData(int32_t buildingIndex);
    bool isBuildingDataUsable(int32_t buildingIndex) const;
    float getBuildingXPos(int32_t buildingIndex) const;
    float getBuildingYPos(int32_t buildingIndex) const;
    float getBuildingAngle(int32_t buildingIndex) const;
    int32_t getNumBuildings() const;
    double getFitness() const;

    bool operator==(const Solution& other);
    bool operator!=(const Solution& other);
  private:
    eastl::vector<float> genes;
    eastl::vector<bool> genesAssignmentStatus;
    int32_t numBuildings;
    double fitness;
    bool hasFitnessSet;
  };
}

#endif
