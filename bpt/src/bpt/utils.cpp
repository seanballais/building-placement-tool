#include <bpt/utils.hpp>

namespace bpt
{
  void swapBuildingData(Solution& solution,
                        const int32_t building0Index,
                        const int32_t building1Index)
  {
    const float building0XPos = solution.getBuildingXPos(building0Index);
    const float building0YPos = solution.getBuildingYPos(building0Index);
    const float building0Rot = solution.getBuildingRotation(building0Index);
    const float building1XPos = solution.getBuildingXPos(building1Index);
    const float building1YPos = solution.getBuildingYPos(building1Index);
    const float building1Rot = solution.getBuildingRotation(building1Index);

    solution.setBuildingXPos(building0Index, building1XPos);
    solution.setBuildingYPos(building0Index, building1YPos);
    solution.setBuildingRotation(building0Index, building1Rot);

    solution.setBuildingXPos(building1Index, building0XPos);
    solution.setBuildingYPos(building1Index, building0YPos);
    solution.setBuildingRotation(building1Index, building0Rot);
  }
}
