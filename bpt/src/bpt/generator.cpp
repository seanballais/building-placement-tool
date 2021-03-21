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
    const float minX = std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](const cx::Point& a, const cx::Point& b) {
        return cx::floatLessThan(a.x, b.x);
      })->x;
    const float maxX = std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](const cx::Point& a, const cx::Point& b) {
        return cx::floatLessThan(a.x, b.x);
      })->x;
    const float minY = std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](const cx::Point& a, const cx::Point& b) {
        return cx::floatLessThan(a.y, b.y);
      })->y;
    const float maxY = std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](const cx::Point& a, const cx::Point& b) {
        return cx::floatLessThan(a.y, b.y);
      })->y;

    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      corex::core::Point buildingPos { 0.f, 0.f };
      buildingPos.x = cx::getRandomRealUniformly(minX, maxX);
      buildingPos.y = cx::getRandomRealUniformly(minY, maxY);
      float buildingRotation = cx::selectItemRandomly(
        eastl::vector<float>{ 0.f, 90.f });

      solution.setBuildingXPos(i, buildingPos.x);
      solution.setBuildingYPos(i, buildingPos.y);
      solution.setBuildingAngle(i, buildingRotation);
    }

    return solution;
  }
}
