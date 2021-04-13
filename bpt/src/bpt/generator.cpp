#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/math_functions.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/operators.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea)
  {
    const float minX = boundingArea.vertices[0].x;
    const float minY = boundingArea.vertices[0].y;
    const float maxX = boundingArea.vertices[2].x;
    const float maxY = boundingArea.vertices[2].y;

    eastl::vector<cx::Rectangle> boundingSubRegions;
    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      corex::core::Point buildingPos { 0.f, 0.f };
      float buildingAngle = 0.f;
      corex::core::Rectangle buildingRect {
        buildingPos.x,
        buildingPos.y,
        inputBuildings[i].width,
        inputBuildings[i].length,
        buildingAngle
      };

      do {
        // We need to get the angle first so that we can correctly get the width
        // and height of the buildingRect.
        buildingAngle = cx::selectItemRandomly(
          eastl::vector<float>{ 0.f, 90.f });
        buildingRect.angle = buildingAngle;

        cx::Polygon<4> poly = cx::rotateRectangle(buildingRect);

        float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
        float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

        buildingPos.x = cx::getRandomRealUniformly(minX + (polyWidth / 2),
                                                   maxX - (polyWidth / 2));
        buildingPos.y = cx::getRandomRealUniformly(minY + (polyHeight / 2),
                                                   maxY - (polyHeight / 2));

        buildingRect.x = buildingPos.x;
        buildingRect.y = buildingPos.y;

        poly = cx::rotateRectangle(buildingRect);

        solution.setBuildingXPos(i, buildingPos.x);
        solution.setBuildingYPos(i, buildingPos.y);
        solution.setBuildingAngle(i, buildingAngle);

        cx::isRectWithinNPolygonAABB(buildingRect, boundingArea);
        doesSolutionHaveNoBuildingsOverlapping(solution,
                                               inputBuildings);
      } while (!cx::isRectWithinNPolygonAABB(buildingRect, boundingArea)
               || !doesSolutionHaveNoBuildingsOverlapping(solution,
                                                          inputBuildings));
    }

    return solution;
  }
}
