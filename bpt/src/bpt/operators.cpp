#include <random>

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/utils.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/operators.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  eastl::vector<Solution> performUniformCrossover(
    const Solution& solutionA,
    const Solution& solutionB,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    int32_t numBuildings = solutionA.getNumBuildings();

    // Prevent unnecessary copying of the parents.
    eastl::array<const Solution* const, 2> parents{ &solutionA, &solutionB };
    eastl::vector<Solution> children{ solutionA, solutionA };
    // Perform Uniform Crossover.
    for (Solution& child : children) {
      do {
        int32_t parentIndex;
        for (int32_t i = 0; i < numBuildings; i++) {
          parentIndex = cx::getRandomIntUniformly(0, 1);
          child.setBuildingXPos(i, parents[parentIndex]->getBuildingXPos(i));

          parentIndex = cx::getRandomIntUniformly(0, 1);
          child.setBuildingYPos(i, parents[parentIndex]->getBuildingYPos(i));

          parentIndex = cx::getRandomIntUniformly(0, 1);
          child.setBuildingAngle(i, parents[parentIndex]->getBuildingAngle(i));
        }
      } while (!keepInfeasibleSolutions
               && !isSolutionFeasible(child, boundingArea, inputBuildings));
    }

    return children;
  }

  eastl::vector<Solution> performBoxCrossover(
    const Solution& solutionA,
    const Solution& solutionB,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    int32_t numBuildings = solutionA.getNumBuildings();

    // Prevent unnecessary copying of the parents.
    eastl::array<const Solution* const, 2> parents{ &solutionA, &solutionB };
    eastl::vector<Solution> children{ solutionA, solutionA };
    for (Solution& child : children) {
      do {
        for (int32_t i = 0; i < numBuildings; i++) {
          // Compute child's new x position.
          float lowerXBound = std::min(parents[0]->getBuildingXPos(i),
                                       parents[1]->getBuildingXPos(i));
          float upperXBound = std::max(parents[0]->getBuildingXPos(i),
                                       parents[1]->getBuildingXPos(i));
          child.setBuildingXPos(
            i,
            cx::getRandomRealUniformly(lowerXBound, upperXBound));

          // Compute child's new y position.
          float lowerYBound = std::min(parents[0]->getBuildingYPos(i),
                                       parents[1]->getBuildingYPos(i));
          float upperYBound = std::max(parents[0]->getBuildingYPos(i),
                                       parents[1]->getBuildingYPos(i));
          child.setBuildingYPos(
            i,
            cx::getRandomRealUniformly(lowerYBound, upperYBound));

          // Compute child's new angle.
          child.setBuildingAngle(
            i,
            cx::selectItemRandomly(eastl::vector<float>{ 0.f, 90.f }));
        }
      } while (!keepInfeasibleSolutions
               && !isSolutionFeasible(child, boundingArea, inputBuildings));
    }

    return children;
  }

  void applyBuddyBuddyOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const int32_t staticBuildingIndex,
    const int32_t dynamicBuildingIndex,
    const bool& keepInfeasibleSolutions)
  {
    std::uniform_int_distribution<int32_t> buildingDistrib{
      0, static_cast<int32_t>(inputBuildings.size() - 1)
    };
    std::uniform_int_distribution<int32_t> buddySideDistrib{ 0, 3 };
    std::uniform_int_distribution<int32_t> relOrientationDistrib{ 0, 1 };
    std::uniform_real_distribution<float> normalizedDistrib{ 0, 1 };

    // Let's just do the Buddy-Buddy Mutation for now.
    int32_t staticBuddy = staticBuildingIndex;
    int32_t dynamicBuddy = dynamicBuildingIndex; // The buddy to be moved.
    if (staticBuddy == -1 || dynamicBuddy == -1) {
      do {
        if (staticBuildingIndex == -1) {
          staticBuddy = corex::core::generateRandomInt(buildingDistrib);
        }

        if (dynamicBuildingIndex == -1) {
          dynamicBuddy = corex::core::generateRandomInt(buildingDistrib);
        }
      } while (staticBuddy == dynamicBuddy
               || !solution.isBuildingDataUsable(staticBuddy)
               || !solution.isBuildingDataUsable(dynamicBuddy));
    }

    auto staticBuddyRect = corex::core::Rectangle{
      solution.getBuildingXPos(staticBuddy),
      solution.getBuildingYPos(staticBuddy),
      inputBuildings[staticBuddy].width,
      inputBuildings[staticBuddy].length,
      solution.getBuildingAngle(staticBuddy)
    };
    auto buddyPoly = corex::core::convertRectangleToPolygon(staticBuddyRect);

    const int32_t buddySide = corex::core::generateRandomInt(
      buddySideDistrib);

    corex::core::Line contactLine;
    switch (buddySide) {
      case 0:
        contactLine = corex::core::Line{
          { buddyPoly.vertices[0].x, buddyPoly.vertices[0].y },
          { buddyPoly.vertices[1].x, buddyPoly.vertices[1].y }
        };
        break;
      case 1:
        contactLine = corex::core::Line{
          { buddyPoly.vertices[1].x, buddyPoly.vertices[1].y },
          { buddyPoly.vertices[2].x, buddyPoly.vertices[2].y }
        };
        break;
      case 2:
        contactLine = corex::core::Line{
          { buddyPoly.vertices[2].x, buddyPoly.vertices[2].y },
          { buddyPoly.vertices[3].x, buddyPoly.vertices[3].y }
        };
        break;
      case 3:
        contactLine = corex::core::Line{
          { buddyPoly.vertices[3].x, buddyPoly.vertices[3].y },
          { buddyPoly.vertices[0].x, buddyPoly.vertices[0].y }
        };
        break;
      default:
        break;
    }

    auto contactLineVec = corex::core::lineToVec(contactLine);
    const int32_t orientation = corex::core::generateRandomInt(
      relOrientationDistrib);
    float distContactToBuddyCenter = 0.f;

    // Length to add to both ends of the contact line vector to allow the
    // edges in the dynamic buddy perpendicular to contact line to be in line
    // with those edges parallel to it in the static buddy.
    float extLength = 0.f;

    float contactLineAngle = corex::core::vec2Angle(contactLineVec);
    float dynamicBuddyAngle;
    if (orientation == 0) {
      // The dynamic buddy will be oriented parallel to the contact line, if
      // width > length. Perpendicular, otherwise.
      distContactToBuddyCenter = inputBuildings[dynamicBuddy].width / 2.f;
      extLength = inputBuildings[dynamicBuddy].length / 2.f;
      dynamicBuddyAngle = contactLineAngle;
    } else {
      // The dynamic buddy will be oriented perpendicular to the contact line,
      // if length > width. Parallel, otherwise.
      distContactToBuddyCenter = inputBuildings[dynamicBuddy].length / 2.f;
      extLength = inputBuildings[dynamicBuddy].width / 2.f;
      dynamicBuddyAngle = contactLineAngle + 90.f;
    }

    // Adjust the distance of the dynamic buddy centroid to the contact line
    // by a small amount to prevent intersection of buildings.
    distContactToBuddyCenter += 0.0001f;

    auto buddyMidptRelContactLine = cx::rotateVec2(
      cx::Vec2{0.f, extLength * 2 },
      contactLineAngle) + contactLineVec;
    auto buddyMidptRelContactLineStart = cx::rotateVec2(
      corex::core::Vec2{ 0.f, -extLength },
      contactLineAngle) + contactLine.start;

    const float lineWidthModifier = corex::core::generateRandomReal(
      normalizedDistrib);

    corex::core::Point dynamicBuddyPos{
      ((buddyMidptRelContactLine * lineWidthModifier)
       + corex::core::vec2Perp(
        corex::core::rotateVec2(
          corex::core::Vec2{ 0.f, distContactToBuddyCenter },
          contactLineAngle)))
      + buddyMidptRelContactLineStart
    };

    solution.setBuildingXPos(dynamicBuddy, dynamicBuddyPos.x);
    solution.setBuildingYPos(dynamicBuddy, dynamicBuddyPos.y);
    solution.setBuildingAngle(dynamicBuddy, dynamicBuddyAngle);
  }

  void applyShakingOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    std::uniform_int_distribution<int32_t> geneDistribution{
      0, solution.getNumBuildings() - 1
    };

    int32_t targetGeneIndex = corex::core::generateRandomInt(geneDistribution);

    float minX = std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.x < ptB.x;
      }
    )->x;
    float maxX = std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.x < ptB.x;
      }
    )->x;
    float minY = std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.y < ptB.y;
      }
    )->y;
    float maxY = std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.y < ptB.y;
      }
    )->y;

    std::uniform_real_distribution<float> xPosDistribution{ minX, maxX };
    std::uniform_real_distribution<float> yPosDistribution{ minY, maxY };

    float newXPos = corex::core::generateRandomReal(xPosDistribution);
    float newYPos = corex::core::generateRandomReal(yPosDistribution);
    float newRotation = cx::selectItemRandomly(
      eastl::vector<float>{ 0.f, 90.f });

    solution.setBuildingXPos(targetGeneIndex, newXPos);
    solution.setBuildingYPos(targetGeneIndex, newYPos);
    solution.setBuildingAngle(targetGeneIndex, newRotation);
  }

  void applyJiggleOperator(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    constexpr int32_t numMovements = 8;
    constexpr float maxShiftAmount = 1.f;
    constexpr float maxRotShiftAmount = 5.f;
    std::uniform_real_distribution<float> shiftDistrib{ 0, maxShiftAmount };
    std::uniform_int_distribution<int32_t> buildingIndexDistrib{
      0, static_cast<int32_t>(inputBuildings.size() - 1)
    };
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t)>,
      numMovements> jiggleFunctions = {
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 - shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 + shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 - shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 + shiftAmountB);
        return solution;
      }
    };
    std::uniform_int_distribution<int32_t> jiggleFuncDistrib{
      0, static_cast<int32_t>(jiggleFunctions.size() - 1)
    };

    const int32_t targetBuildingIndex = cx::generateRandomInt(
      buildingIndexDistrib);
    const int32_t jiggleFuncIndex = cx::generateRandomInt(jiggleFuncDistrib);

    solution = jiggleFunctions[jiggleFuncIndex](solution,
                                                targetBuildingIndex);
    solution.setBuildingAngle(
      targetBuildingIndex,
      cx::selectItemRandomly(eastl::vector<float>{ 0.f, 90.f }));
  }
}
