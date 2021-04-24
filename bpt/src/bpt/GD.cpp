#include <cmath>
#include <iostream>

#include <EASTL/vector.h>
#include <iprof/iprof.hpp>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/Timer.hpp>
#include <corex/core/utils.hpp>

#include <bpt/ds/InputBuilding.hpp>

#include <bpt/GD.hpp>
#include <bpt/evaluator.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  Solution GD::generateSolution(
    Solution initialSolution,
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const double timeLimit,
    const float buildingDistanceWeight,
    const float minExpectedPenalty,
    const float minDecayRate,
    const float maxDecayRate,
    const float decayMultiplier,
    const double minWaterLevelIncrease,
    const double maxWaterLevelIncrease,
    const bool keepInfeasibleSolutions)
  {
    Solution bestSolution = initialSolution;
    double waterLevel = initialSolution.getFitness();

    cx::Timer timer;
    timer.start();
    while (timer.getElapsedTime() <= timeLimit) {
      std::cout << timer.getElapsedTime() << " | " << timeLimit << "\n";
      Solution candidateSolution = initialSolution;
      this->perturbSolution(candidateSolution, boundingArea, inputBuildings,
                            flowRates, floodProneAreas, landslideProneAreas,
                            floodProneAreaPenalty, landslideProneAreaPenalty,
                            buildingDistanceWeight, keepInfeasibleSolutions);
      std::cout << "Candidate Fitness: "
                << candidateSolution.getFitness() << "\n";
      std::cout << "Best Fitness: " << bestSolution.getFitness() << "\n";
      std::cout << "Water Level: " << waterLevel << "\n";
      if (candidateSolution.getFitness() <= bestSolution.getFitness()) {
        initialSolution = candidateSolution;
        bestSolution = initialSolution;
      }

      double range = waterLevel - candidateSolution.getFitness();
      if (range < 1.0) {
        waterLevel += cx::getRandomRealUniformly(minWaterLevelIncrease,
                                                 maxWaterLevelIncrease);
      } else {
        waterLevel = (
          waterLevel
          * std::exp(-decayMultiplier
                     * cx::getRandomRealUniformly(minDecayRate, maxDecayRate))
          + minExpectedPenalty
        );
      }
    }

    timer.stop();

    return bestSolution;
  }

  void GD::perturbSolution(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const bool keepInfeasibleSolutions)
  {
    IPROF_FUNC;
    eastl::array<eastl::function<void(Solution&,
                                      const corex::core::NPolygon&,
                                      const eastl::vector<InputBuilding>&,
                                      const bool&)>,
      3> mutationFunctions = {
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings,
             const bool& keepInfeasibleSolutions)
      {
        this->applyBuddyBuddyMove(solution, boundingArea,
                                  inputBuildings, -1, -1,
                                  keepInfeasibleSolutions);
      },
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings,
             const bool& keepInfeasibleSolutions)
      {
        this->applyShakingMove(solution, boundingArea,
                               inputBuildings, keepInfeasibleSolutions);
      },
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings,
             const bool& keepInfeasibleSolutions)
      {
        this->applyJiggleMove(solution, boundingArea,
                              inputBuildings, keepInfeasibleSolutions);
      }
    };

    Solution tempSolution;
    do {
      tempSolution = solution;
      const int32_t mutationFuncIndex = cx::getRandomIntUniformly(
        0, static_cast<int32_t>(mutationFunctions.size() - 1));
      mutationFunctions[mutationFuncIndex](tempSolution,
                                           boundingArea,
                                           inputBuildings,
                                           keepInfeasibleSolutions);
    } while (!keepInfeasibleSolutions
             && !isSolutionFeasible(tempSolution, boundingArea,
                                    inputBuildings));
    solution = tempSolution;
    solution.setFitness(computeSolutionFitness(solution,
                                               inputBuildings,
                                               boundingArea,
                                               flowRates,
                                               buildingDistanceWeight));
  }

  void GD::applyBuddyBuddyMove(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const int32_t staticBuildingIndex,
    const int32_t dynamicBuildingIndex,
    const bool& keepInfeasibleSolutions)
  {
    IPROF_FUNC;
    std::uniform_int_distribution<int32_t> buildingDistrib{
      0, static_cast<int32_t>(inputBuildings.size() - 1)
    };
    std::uniform_int_distribution<int32_t> buddySideDistrib{ 0, 3 };
    std::uniform_int_distribution<int32_t> relOrientationDistrib{ 0, 1 };
    std::uniform_real_distribution<float> normalizedDistrib{ 0, 1 };

    // Let's just do the Buddy-Buddy Mutation for now.
    int32_t staticBuddy = 0;
    int32_t dynamicBuddy = 0; // The buddy to be moved.
    do {
      if (staticBuildingIndex == -1) {
        staticBuddy = corex::core::generateRandomInt(buildingDistrib);
      } else {
        staticBuddy = staticBuildingIndex;
      }

      if (dynamicBuildingIndex == -1) {
        dynamicBuddy = corex::core::generateRandomInt(buildingDistrib);
      } else {
        dynamicBuddy = dynamicBuildingIndex;
      }
    } while (staticBuddy == dynamicBuddy
             || !solution.isBuildingDataUsable(staticBuddy)
             || !solution.isBuildingDataUsable(dynamicBuddy));

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
    } else if (orientation == 1) {
      // The dynamic buddy will be oriented perpendicular to the contact line,
      // if length > width. Parallel, otherwise.
      distContactToBuddyCenter = inputBuildings[dynamicBuddy].length / 2.f;
      extLength = inputBuildings[dynamicBuddy].width / 2.f;
      dynamicBuddyAngle = contactLineAngle + 45.f;
    }

    // Adjust the distance of the dynamic buddy centroid to the contact line
    // by a small amount to prevent intersection of buildings.
    distContactToBuddyCenter += 0.0001f;

    auto buddyMidptRelContactLine = corex::core::rotateVec2(
      corex::core::Vec2{0.f, extLength * 2 }, contactLineAngle)
                                    + contactLineVec;
    auto buddyMidptRelContactLineStart = corex::core::rotateVec2(
      corex::core::Vec2{ 0.f, -extLength }, contactLineAngle)
                                         + contactLine.start;

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

  void GD::applyShakingMove(
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
    std::uniform_real_distribution<float> rotationDistribution{ 0.f, 360.f };

    float newXPos = corex::core::generateRandomReal(xPosDistribution);
    float newYPos = corex::core::generateRandomReal(yPosDistribution);
    float newRotation = corex::core::generateRandomReal(rotationDistribution);

    solution.setBuildingXPos(targetGeneIndex, newXPos);
    solution.setBuildingYPos(targetGeneIndex, newYPos);
    solution.setBuildingAngle(targetGeneIndex, newRotation);
  }

  void GD::applyJiggleMove(
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
    std::uniform_real_distribution<float> rotShiftDistrib{
      -maxRotShiftAmount,
      maxRotShiftAmount
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

    const int32_t targetBuildingIndex = corex::core::generateRandomInt(
      buildingIndexDistrib);
    const int32_t jiggleFuncIndex = corex::core::generateRandomInt(
      jiggleFuncDistrib);

    solution = jiggleFunctions[jiggleFuncIndex](solution,
                                                targetBuildingIndex);

    const float rotDelta = corex::core::generateRandomReal(rotShiftDistrib);
    const float newRot = solution.getBuildingAngle(targetBuildingIndex)
                         + rotDelta;
    solution.setBuildingAngle(targetBuildingIndex, newRot);
  }
}
