#include <random>

#include <EASTL/utility.h>
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

  eastl::vector<Solution> performArithmeticCrossover(
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
    for (int32_t i = 0; i < children.size(); i++) {
      do {
        float a = cx::getRandomRealUniformly(0.f, 1.f);
        cx::VecN parent0 = convertSolutionToVecN(*parents[0]);
        cx::VecN parent1 = convertSolutionToVecN(*parents[1]);

        eastl::unique_ptr<cx::VecN> childVecPtr;
        if (i == 0) {
          childVecPtr = eastl::make_unique<cx::VecN>(
            (a * parent0) + ((1 - a) * parent1));
        } else if (i == 1) {
          childVecPtr = eastl::make_unique<cx::VecN>(
            (a * parent1) + ((1 - a) * parent0));
        }

        // Fix the mutations parts.
        for (int32_t j = 0; j < numBuildings; j++) {
          float angleVal = (*childVecPtr)[(j * 3) + 2];
          if (cx::floatLessEqual(angleVal, 45.f)) {
            (*childVecPtr)[(j * 3) + 2] = 0.f;
          } else {
            (*childVecPtr)[(j * 3) + 2] = 90.f;
          }
        }

        children[i] = convertVecNToSolution(*childVecPtr);
      } while (!keepInfeasibleSolutions
               && !isSolutionFeasible(children[i],
                                      boundingArea,
                                      inputBuildings));
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
    Solution tempSolution;
    do {
      tempSolution = solution;
      std::uniform_int_distribution<int32_t> buildingDistrib{
        0, static_cast<int32_t>(inputBuildings.size() - 1)
      };
      std::uniform_int_distribution<int32_t> relOrientationDistrib{ 0, 1 };
      std::uniform_real_distribution<float> normalizedDistrib{ 0, 1 };

      // Let's just do the Buddy-Buddy Mutation for now.
      int32_t staticBuddy = staticBuildingIndex;
      int32_t dynamicBuddy = dynamicBuildingIndex; // The buddy to be moved.
      if (staticBuddy == -1 || dynamicBuddy == -1) {
        do {
          if (staticBuildingIndex == -1) {
            staticBuddy = cx::generateRandomInt(buildingDistrib);
          }

          if (dynamicBuildingIndex == -1) {
            // Give higher priority to buildings that intersect with other
            // buildings.
            eastl::vector<int32_t> buildingIDs;
            eastl::vector<float> buildingSelectionWeights;
            for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
              buildingIDs.push_back(i);

              if (!solution.isBuildingDataUsable(i)) {
                buildingSelectionWeights.push_back(0.f);
                continue;
              }

              cx::Rectangle building0{
                solution.getBuildingXPos(i),
                solution.getBuildingYPos(i),
                inputBuildings[i].width,
                inputBuildings[i].length,
                solution.getBuildingAngle(i)
              };

              bool hasIntersectingBuilding = false;
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
                  hasIntersectingBuilding = true;
                  break;
                }
              }

              if (hasIntersectingBuilding) {
                buildingSelectionWeights.push_back(50.f);
              } else {
                buildingSelectionWeights.push_back(5.f);
              }
            }

            dynamicBuddy = cx::selectRandomItemWithWeights(
              buildingIDs, buildingSelectionWeights);
          }
        } while (staticBuddy == dynamicBuddy
                 || !tempSolution.isBuildingDataUsable(staticBuddy)
                 || !tempSolution.isBuildingDataUsable(dynamicBuddy));
      }

      auto staticBuddyRect = cx::Rectangle{
        tempSolution.getBuildingXPos(staticBuddy),
        tempSolution.getBuildingYPos(staticBuddy),
        inputBuildings[staticBuddy].width,
        inputBuildings[staticBuddy].length,
        tempSolution.getBuildingAngle(staticBuddy)
      };
      auto staticBuddyPoly = cx::convertRectangleToPolygon(staticBuddyRect);

      // Determine which sides of the polygon we can put the dynamic buddy.
      const float orientation = cx::selectItemRandomly(
        eastl::vector<float>{ 0.f, 90.f });

      const int32_t buddySide = cx::getRandomIntUniformly(0, 3);

      corex::core::Line contactLine;
      switch (buddySide) {
        case 0:
          // Top line.
          contactLine = corex::core::Line{
            {staticBuddyPoly.vertices[0].x, staticBuddyPoly.vertices[0].y },
            {staticBuddyPoly.vertices[1].x, staticBuddyPoly.vertices[1].y }
          };
          break;
        case 1:
          // Right-side line.
          contactLine = corex::core::Line{
            {staticBuddyPoly.vertices[1].x, staticBuddyPoly.vertices[1].y },
            {staticBuddyPoly.vertices[2].x, staticBuddyPoly.vertices[2].y }
          };
          break;
        case 2:
          // Bottom line.
          contactLine = corex::core::Line{
            {staticBuddyPoly.vertices[2].x, staticBuddyPoly.vertices[2].y },
            {staticBuddyPoly.vertices[3].x, staticBuddyPoly.vertices[3].y }
          };
          break;
        case 3:
          // Left-side line.
          contactLine = corex::core::Line{
            {staticBuddyPoly.vertices[3].x, staticBuddyPoly.vertices[3].y },
            {staticBuddyPoly.vertices[0].x, staticBuddyPoly.vertices[0].y }
          };
          break;
        default:
          break;
      }

      auto contactLineVec = cx::lineToVec(contactLine);
      float distContactToBuddyCenter = 0.f;

      // Length to add to both ends of the contact line vector to allow the
      // edges in the dynamic buddy perpendicular to contact line to be in line
      // with those edges parallel to it in the static buddy.
      float extLength;

      float contactLineAngle = corex::core::vec2Angle(contactLineVec);
      float dynamicBuddyAngle;
      if (cx::floatEquals(orientation, 0.f)) {
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

      cx::Point dynamicBuddyPos{
        ((buddyMidptRelContactLine * lineWidthModifier)
         + corex::core::vec2Perp(
          corex::core::rotateVec2(
            corex::core::Vec2{ 0.f, distContactToBuddyCenter },
            contactLineAngle)))
        + buddyMidptRelContactLineStart
      };

      auto dynamicBuddyRect = cx::Rectangle{
        dynamicBuddyPos.x,
        dynamicBuddyPos.y,
        inputBuildings[dynamicBuddy].width,
        inputBuildings[dynamicBuddy].length,
        dynamicBuddyAngle
      };

      if (!cx::isRectWithinNPolygonAABB(dynamicBuddyRect, boundingArea)) {
        continue;
      }

      tempSolution.setBuildingXPos(dynamicBuddy, dynamicBuddyPos.x);
      tempSolution.setBuildingYPos(dynamicBuddy, dynamicBuddyPos.y);
      tempSolution.setBuildingAngle(dynamicBuddy, dynamicBuddyAngle);
    } while ((!keepInfeasibleSolutions
              && !isSolutionFeasible(tempSolution,
                                     boundingArea,
                                     inputBuildings)));

    solution = tempSolution;
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

  void applyOrientationFlipping(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    int32_t buildingIdx = cx::getRandomIntUniformly(0,
                                                    inputBuildings.size() - 1);
    float normAngle = solution.getBuildingAngle(buildingIdx) / 90.f;
    float randValue = cx::getRandomRealNormDistrib(0.f, 1.f);

    if (cx::floatGreEqual(normAngle + randValue, 0.5f)) {
      solution.setBuildingAngle(buildingIdx, 90.f);
    } else {
      solution.setBuildingAngle(buildingIdx, 0.f);
    }
  }

  void applySwappingMethod(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    constexpr int32_t numFuncs = 7;
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t, int32_t)>,
      numFuncs> swappingFunctions = {
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Change the orientation of building 0.
        if (cx::floatEquals(solution.getBuildingAngle(building0Index), 0.f)) {
          solution.setBuildingAngle(building0Index, 90.f);
        } else {
          solution.setBuildingAngle(building0Index, 0.f);
        }

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Change the orientation of building 1.
        if (cx::floatEquals(solution.getBuildingAngle(building1Index), 0.f)) {
          solution.setBuildingAngle(building1Index, 90.f);
        } else {
          solution.setBuildingAngle(building1Index, 0.f);
        }

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Change the orientation of building 0 and 1.
        if (cx::floatEquals(solution.getBuildingAngle(building0Index), 0.f)) {
          solution.setBuildingAngle(building0Index, 90.f);
        } else {
          solution.setBuildingAngle(building0Index, 0.f);
        }

        if (cx::floatEquals(solution.getBuildingAngle(building1Index), 0.f)) {
          solution.setBuildingAngle(building1Index, 90.f);
        } else {
          solution.setBuildingAngle(building1Index, 0.f);
        }

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Swap the locations of the buildings.
        cx::Point building0Pos{
          solution.getBuildingXPos(building0Index),
          solution.getBuildingYPos(building0Index)
        };
        cx::Point building1Pos{
          solution.getBuildingXPos(building1Index),
          solution.getBuildingYPos(building1Index)
        };

        eastl::swap(building0Pos, building1Pos);

        solution.setBuildingXPos(building0Index, building0Pos.x);
        solution.setBuildingYPos(building0Index, building0Pos.y);

        solution.setBuildingXPos(building1Index, building1Pos.x);
        solution.setBuildingYPos(building1Index, building1Pos.y);

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Swap the locations of the buildings and change the orientation of
        // building 0.
        cx::Point building0Pos{
          solution.getBuildingXPos(building0Index),
          solution.getBuildingYPos(building0Index)
        };
        cx::Point building1Pos{
          solution.getBuildingXPos(building1Index),
          solution.getBuildingYPos(building1Index)
        };

        eastl::swap(building0Pos, building1Pos);

        solution.setBuildingXPos(building0Index, building0Pos.x);
        solution.setBuildingYPos(building0Index, building0Pos.y);

        solution.setBuildingXPos(building1Index, building1Pos.x);
        solution.setBuildingYPos(building1Index, building1Pos.y);

        if (cx::floatEquals(solution.getBuildingAngle(building0Index), 0.f)) {
          solution.setBuildingAngle(building0Index, 90.f);
        } else {
          solution.setBuildingAngle(building0Index, 0.f);
        }

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Swap the locations of the buildings and change the orientation of
        // building 1.
        cx::Point building0Pos{
          solution.getBuildingXPos(building0Index),
          solution.getBuildingYPos(building0Index)
        };
        cx::Point building1Pos{
          solution.getBuildingXPos(building1Index),
          solution.getBuildingYPos(building1Index)
        };

        eastl::swap(building0Pos, building1Pos);

        solution.setBuildingXPos(building0Index, building0Pos.x);
        solution.setBuildingYPos(building0Index, building0Pos.y);

        solution.setBuildingXPos(building1Index, building1Pos.x);
        solution.setBuildingYPos(building1Index, building1Pos.y);

        if (cx::floatEquals(solution.getBuildingAngle(building1Index), 0.f)) {
          solution.setBuildingAngle(building1Index, 90.f);
        } else {
          solution.setBuildingAngle(building1Index, 0.f);
        }

        return solution;
      },
      [](Solution solution,
         int32_t building0Index,
         int32_t building1Index) -> Solution
      {
        // Swap the locations of the buildings and change the orientation of
        // building 0 and 1.
        cx::Point building0Pos{
          solution.getBuildingXPos(building0Index),
          solution.getBuildingYPos(building0Index)
        };
        cx::Point building1Pos{
          solution.getBuildingXPos(building1Index),
          solution.getBuildingYPos(building1Index)
        };

        eastl::swap(building0Pos, building1Pos);

        solution.setBuildingXPos(building0Index, building0Pos.x);
        solution.setBuildingYPos(building0Index, building0Pos.y);

        solution.setBuildingXPos(building1Index, building1Pos.x);
        solution.setBuildingYPos(building1Index, building1Pos.y);

        if (cx::floatEquals(solution.getBuildingAngle(building0Index), 0.f)) {
          solution.setBuildingAngle(building0Index, 90.f);
        } else {
          solution.setBuildingAngle(building0Index, 0.f);
        }

        if (cx::floatEquals(solution.getBuildingAngle(building1Index), 0.f)) {
          solution.setBuildingAngle(building1Index, 90.f);
        } else {
          solution.setBuildingAngle(building1Index, 0.f);
        }

        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings() - 1; i++) {
      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        for (int32_t funcID = 0; funcID < numFuncs; funcID++) {
          generatedSolutions.push_back(
            swappingFunctions[funcID](solution, i, j));
        }
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](const Solution& solutionA, const Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );
  }

  eastl::pair<cx::Point, cx::Point>
  getBuildingMinMaxPoints(Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings,
                          const float& buildingIndex)
  {
    const cx::Point& boundingMinPt = boundingArea.vertices[0];
    const cx::Point& boundingMaxPt = boundingArea.vertices[2];

    corex::core::Rectangle buildingRect {
      solution.getBuildingXPos(buildingIndex),
      solution.getBuildingYPos(buildingIndex),
      inputBuildings[buildingIndex].width,
      inputBuildings[buildingIndex].length,
      solution.getBuildingAngle(buildingIndex)
    };

    cx::Polygon<4> poly = cx::rotateRectangle(buildingRect);

    float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
    float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

    float minBuildingXVal = boundingMinPt.x + (polyWidth / 2);
    float maxBuildingXVal = boundingMaxPt.x - (polyWidth / 2);
    float minBuildingYVal = boundingMinPt.y + (polyHeight / 2);
    float maxBuildingYVal = boundingMaxPt.y - (polyHeight / 2);

    if (minBuildingXVal > maxBuildingXVal) {
      eastl::swap(minBuildingXVal, maxBuildingXVal);
    }

    if (minBuildingYVal > maxBuildingYVal) {
      eastl::swap(minBuildingYVal, maxBuildingYVal);
    }

    return eastl::make_pair(cx::Point{ minBuildingXVal, minBuildingYVal },
                            cx::Point{ maxBuildingXVal, maxBuildingYVal });
  }

  void applyLocalSearch1(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    constexpr int32_t numMovements = 8;
    constexpr float maxShiftAmount = 5.f;

    const cx::Point& minPt = boundingArea.vertices[0];
    const cx::Point& maxPt = boundingArea.vertices[2];

    std::uniform_real_distribution<float> shiftDistrib{ 1.f, maxShiftAmount };
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t)>, numMovements>
    searchFunctions = {
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  shiftAmount,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  -shiftAmount,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the top.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  -shiftAmount,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the bottom.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  shiftAmount,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right then upward.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  shiftAmount0,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        minMaxPoints = getBuildingMinMaxPoints(solution,
                                               boundingArea,
                                               inputBuildings,
                                               buildingIndex);
        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  -shiftAmount1,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right and then downward.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);

        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  shiftAmount0,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        minMaxPoints = getBuildingMinMaxPoints(solution,
                                               boundingArea,
                                               inputBuildings,
                                               buildingIndex);
        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  shiftAmount1,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left and then upward.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  -shiftAmount0,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        minMaxPoints = getBuildingMinMaxPoints(solution,
                                               boundingArea,
                                               inputBuildings,
                                               buildingIndex);
        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  -shiftAmount1,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left and then downward.
        auto minMaxPoints = getBuildingMinMaxPoints(solution,
                                                    boundingArea,
                                                    inputBuildings,
                                                    buildingIndex);

        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex,
                                  shiftAmount0,
                                  minMaxPoints.first.x,
                                  minMaxPoints.second.x);

        minMaxPoints = getBuildingMinMaxPoints(solution,
                                               boundingArea,
                                               inputBuildings,
                                               buildingIndex);
        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex,
                                  shiftAmount1,
                                  minMaxPoints.first.y,
                                  minMaxPoints.second.y);

        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      for (int32_t movementID = 0; movementID < numMovements; movementID++) {
        auto altSolution0 = searchFunctions[movementID](solution, i);
        altSolution0.setFitness(
          computeSolutionFitness(altSolution0,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution0);

        auto altSolution1 = altSolution0;
        if (cx::floatEquals(altSolution1.getBuildingAngle(i), 0.f)) {
          altSolution1.setBuildingAngle(i, 90.f);
        } else {
          altSolution1.setBuildingAngle(i, 0.f);
        }
        altSolution1.setFitness(
          computeSolutionFitness(altSolution1,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));

        generatedSolutions.push_back(altSolution1);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](const Solution& solutionA, const Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );
  }

  void applyLocalSearch2(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    constexpr int32_t numMovements = 20;
    constexpr float maxShiftAmount = 5.f;
    std::uniform_real_distribution<float> shiftDistrib{ 0.f, maxShiftAmount };
    static const
    eastl::array<
      eastl::function<Solution(Solution, int32_t, int32_t)>, numMovements>
    searchFunctions = {
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the right side.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the left side.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount,
                                  minMaxPoints0.first.x,
                                  minMaxPoints1.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings upwards.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings downwards.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the top-right.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount0,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount1,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the bottom-right.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount0,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount1,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the top-left.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount0,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount1,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift both buildings to the bottom-left.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount0,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount1,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 upward and building 1 to the right.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 upward and building 1 downward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 upward and building 1 to the left.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the right and building 1 downward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints0.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the right and building 1 upward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the right and building 1 to the left.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the left and building 1 downward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the left and building 1 to the right.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 to the left and building 1 upward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx,
                                  -shiftAmount0,
                                  minMaxPoints0.first.x,
                                  minMaxPoints0.second.x);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 downward and building 1 to the right.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 downward and building 1 to the left.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.x,
                                  minMaxPoints1.second.x);

        return solution;
      },
      [&shiftDistrib, &boundingArea, &inputBuildings]
      (Solution solution, int32_t building0Idx, int32_t building1Idx)
      -> Solution
      {
        // Shift building 0 downward and building 1 upward.
        auto minMaxPoints0 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building0Idx);
        auto minMaxPoints1 = getBuildingMinMaxPoints(solution,
                                                     boundingArea,
                                                     inputBuildings,
                                                     building1Idx);
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx,
                                  shiftAmount0,
                                  minMaxPoints0.first.y,
                                  minMaxPoints0.second.y);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx,
                                  -shiftAmount1,
                                  minMaxPoints1.first.y,
                                  minMaxPoints1.second.y);

        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings() - 1; i++) {
      for (int32_t movementID = 0; movementID < numMovements; movementID++) {
        auto altSolution0 = searchFunctions[movementID](solution, i, i + 1);
        altSolution0.setFitness(
          computeSolutionFitness(altSolution0,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution0);

        auto altSolution1 = altSolution0;
        auto altSolution2 = altSolution0;
        auto altSolution3 = altSolution0;

        if (cx::floatEquals(altSolution1.getBuildingAngle(i), 0.f)) {
          altSolution1.setBuildingAngle(i, 90.f);
        } else {
          altSolution1.setBuildingAngle(i, 0.f);
        }

        if (cx::floatEquals(altSolution2.getBuildingAngle(i + 1), 0.f)) {
          altSolution2.setBuildingAngle(i + 1, 90.f);
        } else {
          altSolution2.setBuildingAngle(i + 1, 0.f);
        }

        if (cx::floatEquals(altSolution3.getBuildingAngle(i), 0.f)) {
          altSolution3.setBuildingAngle(i, 90.f);
        } else {
          altSolution3.setBuildingAngle(i, 0.f);
        }

        if (cx::floatEquals(altSolution3.getBuildingAngle(i + 1), 0.f)) {
          altSolution3.setBuildingAngle(i + 1, 90.f);
        } else {
          altSolution3.setBuildingAngle(i + 1, 0.f);
        }

        altSolution1.setFitness(
          computeSolutionFitness(altSolution1,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution1);

        altSolution2.setFitness(
          computeSolutionFitness(altSolution2,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution2);

        altSolution3.setFitness(
          computeSolutionFitness(altSolution3,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution3);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](const Solution& solutionA, const Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );
  }
}
