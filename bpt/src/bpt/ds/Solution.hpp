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
    Solution(int32_t numBuildings, Solution* parentA, Solution* parentB);

    void setBuildingXPos(int32_t buildingIndex, float xPos);
    void setBuildingYPos(int32_t buildingIndex, float yPos);
    void setBuildingRotation(int32_t buildingIndex, float rotation);
    void setFitness(double fitness);
    float getBuildingXPos(int32_t buildingIndex) const;
    float getBuildingYPos(int32_t buildingIndex) const;
    float getBuildingRotation(int32_t buildingIndex) const;
    int32_t getNumBuildings() const;
    double getFitness() const;
    Solution* const getParentA();
    Solution* const getParentB();

    bool operator==(const Solution& other);
    bool operator!=(const Solution& other);
  private:
    eastl::vector<float> genes;
    int32_t numBuildings;
    double fitness;
    bool hasFitnessSet;

    // Not setting the pointers below to const because it'll implicitly delete
    // the copy constructor, which is used a lot in the GA code. Note that these
    // pointers were added after the GA code was written. Sure, I can go ahead
    // and refactor the GA code to use unique pointers to let me be able to set
    // the pointers below to const, but that may introduce bugs and take up time
    // that would be better used to finish the GA, and meet the deadline for
    // this thesis.
    Solution* parentA;
    Solution* parentB;
  };
}

#endif
