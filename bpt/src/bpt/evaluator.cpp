#include <corex/core/math_functions.hpp>
#include <corex/core/ds/Rectangle.hpp>

#include <bpt/evaluator.hpp>

namespace bpt
{
  double computeSolutionFitness(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    double fitness = 0.0;

    // Compute fitness for the inter-building distance part.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      assert(flowRates[i].size() == solution.getNumBuildings());
      for (int32_t j = 1; j < solution.getNumBuildings(); j++) {
        if (i == j) {
          continue;
        }

        float xi = solution.getBuildingXPos(i);
        float xj = solution.getBuildingXPos(j);
        float yi = solution.getBuildingYPos(i);
        float yj = solution.getBuildingYPos(j);

        fitness += (cx::abs(xi - xj) + cx::abs(yi - yj)) * flowRates[i][j];
      }
    }

    fitness *= buildingDistanceWeight;

    // Compute penalty for infeasible solutions.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      if (!solution.isBuildingDataUsable(i)) {
        continue;
      }

      corex::core::Rectangle building0{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingAngle(i)
      };

      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        if (!solution.isBuildingDataUsable(j)) {
          continue;
        }

        corex::core::Rectangle building1{
          solution.getBuildingXPos(j),
          solution.getBuildingYPos(j),
          inputBuildings[j].width,
          inputBuildings[j].length,
          solution.getBuildingAngle(j)
        };
        if (corex::core::areTwoRectsAABBIntersecting(building0, building1)) {
          // Let's penalize buildings based on how much of the smaller building
          // is being intersected by the bigger building.
          auto intersectingPoly = cx::getIntersectingRectAABB(building0,
                                                              building1);
          cx::Point& minPt = intersectingPoly.vertices[0];
          cx::Point& maxPt = intersectingPoly.vertices[2];
          float width = maxPt.x - minPt.x;
          float height = maxPt.y - minPt.y;
          double intersectionArea = width * height;

          double building0Area = building0.width * building0.height;
          double building1Area = building1.width * building1.height;

          constexpr double penaltyVal = 1000000.0;
          double baseArea = (building0Area < building1Area) ? building0Area
                                                            : building1Area;
          fitness += (penaltyVal * (intersectionArea / baseArea)) + penaltyVal;
        }
      }
    }

    return fitness;
  }

  bool isSolutionFeasible(const Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings)
  {
    return doesSolutionHaveNoBuildingsOverlapping(solution, inputBuildings)
           && areSolutionBuildingsWithinBounds(solution,
                                               boundingArea,
                                               inputBuildings);
  }

  bool doesSolutionHaveNoBuildingsOverlapping(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      if (!solution.isBuildingDataUsable(i)) {
        continue;
      }

      corex::core::Rectangle building0{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingAngle(i)
      };

      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        if (!solution.isBuildingDataUsable(j)) {
          continue;
        }

        corex::core::Rectangle building1{
          solution.getBuildingXPos(j),
          solution.getBuildingYPos(j),
          inputBuildings[j].width,
          inputBuildings[j].length,
          solution.getBuildingAngle(j)
        };
        if (cx::areTwoRectsAABBIntersecting(building0, building1)) {
          return false;
        }
      }
    }

    return true;
  }

  bool areSolutionBuildingsWithinBounds(
    const Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      if (!solution.isBuildingDataUsable(i)) {
        continue;
      }

      corex::core::Rectangle buildingRect{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingAngle(i)
      };

      if (!corex::core::isRectWithinNPolygonAABB(buildingRect, boundingArea)) {
        return false;
      }
    }

    return true;
  }
}
