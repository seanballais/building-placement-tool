#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/math_functions.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>
#include <bpt/evaluator.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea)
  {
    std::uniform_real_distribution<float> rotationDistribution{ 0.f, 360.f };
    auto boundingAreaTriangles = cx::earClipTriangulate(boundingArea);
    eastl::vector<float> triangleAreas(boundingAreaTriangles.size());

    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      do {
        corex::core::Point buildingPos { 0.f, 0.f };
        float buildingRotation = 0.f;
        corex::core::Rectangle buildingRect {
          buildingPos.x,
          buildingPos.y,
          inputBuildings[i].width,
          inputBuildings[i].length,
          buildingRotation
        };

        const cx::Polygon<3>& triangle = cx::selectRandomItemWithWeights(
          boundingAreaTriangles,
          triangleAreas);
        cx::Point newBuildingPos = cx::getRandomPointInTriangle(triangle);

        buildingPos.x = newBuildingPos.x;
        buildingPos.y = newBuildingPos.y;
        buildingRotation = cx::selectItemRandomly(
          eastl::vector<float>{ 0.f, 90.f });
        buildingRect.x = buildingPos.x;
        buildingRect.y = buildingPos.y;
        buildingRect.angle = buildingRotation;

        solution.setBuildingXPos(i, buildingPos.x);
        solution.setBuildingYPos(i, buildingPos.y);
        solution.setBuildingAngle(i, buildingRotation);
      } while (!areSolutionBuildingsWithinBounds(solution,
                                                 boundingArea,
                                                 inputBuildings));
    }

    return solution;
  }
}
