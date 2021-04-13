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
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
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

        fitness += static_cast<double>(
          corex::core::distance2D(corex::core::Point{
                                    solution.getBuildingXPos(i),
                                    solution.getBuildingYPos(i)
                                  },
                                  corex::core::Point{
                                    solution.getBuildingXPos(j),
                                    solution.getBuildingYPos(j)
                                  })
          * flowRates[i][j]
        );
      }
    }

    fitness *= buildingDistanceWeight;

    // Compute penalty for placing buildings in hazard areas.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      corex::core::Rectangle building {
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingAngle(i)
      };
      // Compute penalty for placing a building in a flood-prone area.
      for (const corex::core::NPolygon& area : floodProneAreas) {
        if (corex::core::isRectIntersectingNPolygon(building, area)) {
          fitness += floodProneAreaPenalty;
        }
      }

      // Compute penalty for placing a building in a landslide-prone area.
      for (const corex::core::NPolygon& area : landslideProneAreas) {
        if (corex::core::isRectIntersectingNPolygon(building, area)) {
          fitness += landslideProneAreaPenalty;
        }
      }
    }

    // Compute penalty for infeasible solutions.
    if (!isSolutionFeasible(solution, boundingArea, inputBuildings)) {
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
          if (corex::core::areTwoRectsIntersecting(building0, building1)) {
            fitness += 100000.0;
          }
        }
      }

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

        if (!corex::core::isRectWithinNPolygon(buildingRect, boundingArea)) {
          fitness += 200000.0;
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
        if (cx::areTwoRectsAABBIntersecting(building0, building1)
            || cx::isRectWithinRectAABB(building0, building1)) {
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
