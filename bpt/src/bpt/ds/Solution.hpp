#ifndef BPT_DS_SOLUTION_HPP
#define BPT_DS_SOLUTION_HPP

#include <cstdlib>

#include <EASTL/array.h>
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
    void setBuildingRotation(int32_t buildingIndex, float rotation);
    float getBuildingXPos(int32_t buildingIndex);
    float getBuildingYPos(int32_t buildingIndex);
    float getBuildingRotation(int32_t buildingIndex);
    int32_t getNumBuildings();

    eastl::array<Solution, 2> crossover(Solution& other);
    void mutate();
  private:
    eastl::vector<float> genes;
    int32_t numBuildings;
  };
}

#endif
