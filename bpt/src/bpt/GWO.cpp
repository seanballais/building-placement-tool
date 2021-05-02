#include <cassert>
#include <iostream>

#include <EASTL/algorithm.h>
#include <EASTL/array.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/Timer.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/VecN.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
#include <bpt/GWO.hpp>
#include <bpt/HC.hpp>
#include <bpt/operators.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/GWOResult.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  GWO::GWO()
    : runTimer()
    , currRunIterationNumber(0)
    , hillClimbing() {}

  GWOResult GWO::generateSolutions(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const int32_t numWolves,
    const int32_t numIterations,
    const float alphaDecayRate,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const bool isLocalSearchEnabled,
    const double timeLimit,
    const bool &keepInfeasibleSolutions)
  {
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<Solution> wolves;
    eastl::vector<double> wolfMutationRates;
    eastl::vector<double> bestFitnesses;
    eastl::vector<double> averageFitnesses;
    eastl::vector<double> worstFitnesses;

    // NOTE: wolves and wolfMutationRates must *never* be sorted.

    this->runTimer.start();

    // Generate initial wolves.
    for (int32_t i = 0; i < numWolves; i++) {
      std::cout << "Generating Wolf #" << i << "\n";
      wolves.push_back(generateRandomSolution(inputBuildings,
                                              boundingArea));
      wolfMutationRates.push_back(0.0);
    }

    // Evaluate individual fitness and mutation rate.
    this->computeWolfValues(
      wolves,
      wolfMutationRates,
      inputBuildings,
      boundingArea,
      flowRates,
      floodProneAreas,
      landslideProneAreas,
      floodProneAreaPenalty,
      landslideProneAreaPenalty,
      buildingDistanceWeight);

    double worstFitness = eastl::max_element(
      wolves.begin(),
      wolves.end(),
      [](const Solution& solutionA, const Solution& solutionB) -> bool {
        return solutionA.getFitness() < solutionB.getFitness();
      })->getFitness();
    double bestFitness = eastl::min_element(
      wolves.begin(),
      wolves.end(),
      [](const Solution& solutionA, const Solution& solutionB) -> bool {
        return solutionA.getFitness() < solutionB.getFitness();
      })->getFitness();

    double averageFitness = 0.0;
    for (Solution& wolf : wolves) {
      averageFitness += wolf.getFitness();
    }

    averageFitness /= wolves.size();

    bestFitnesses.push_back(bestFitness);
    averageFitnesses.push_back(averageFitness);
    worstFitnesses.push_back(worstFitness);

    std::sort(
      wolves.begin(),
      wolves.end(),
      [](Solution& solutionA, Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );

    solutions.push_back(wolves);

    float alpha = 2.f;
    for (int32_t i = 0; i < numIterations; i++) {
      this->currRunIterationNumber = i;

      // Update individuals based on a custom search operator.
      this->updateWolves(
        wolves,
        alpha,
        boundingArea,
        inputBuildings,
        keepInfeasibleSolutions);

      // Evaluate individual fitness and mutation rate.
      this->computeWolfValues(
        wolves,
        wolfMutationRates,
        inputBuildings,
        boundingArea,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight);

      // Reduce values of alpha vector.
      alpha = cx::clamp(alpha - alphaDecayRate, 0.f, 1.25f);

      std::sort(
        wolves.begin(),
        wolves.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return solutionA.getFitness() < solutionB.getFitness();
        }
      );

      Solution& currGenBest = *std::min_element(
        wolves.begin(),
        wolves.end(),
        [](const Solution& a, const Solution& b) {
          return a.getFitness() < b.getFitness();
        }
      );

      applyLocalSearch1(currGenBest,
                        boundingArea,
                        inputBuildings,
                        flowRates,
                        buildingDistanceWeight);

      if (i >= numIterations - 50) {
        applyLocalSearch2(currGenBest,
                          boundingArea,
                          inputBuildings,
                          flowRates,
                          buildingDistanceWeight);
      }

      std::sort(
        wolves.begin(),
        wolves.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return solutionA.getFitness() < solutionB.getFitness();
        }
      );

      // Compute statistical data.
      double worstFitness = eastl::max_element(
        wolves.begin(),
        wolves.end(),
        [](const Solution& solutionA, const Solution& solutionB) -> bool {
          return solutionA.getFitness() < solutionB.getFitness();
        })->getFitness();
      double bestFitness = eastl::min_element(
        wolves.begin(),
        wolves.end(),
        [](const Solution& solutionA, const Solution& solutionB) -> bool {
          return solutionA.getFitness() < solutionB.getFitness();
        })->getFitness();

      double averageFitness = 0.0;
      for (Solution& wolf : wolves) {
        averageFitness += wolf.getFitness();
      }

      averageFitness /= wolves.size();

      bestFitnesses.push_back(bestFitness);
      averageFitnesses.push_back(averageFitness);
      worstFitnesses.push_back(worstFitness);

      std::sort(
        wolves.begin(),
        wolves.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return solutionA.getFitness() < solutionB.getFitness();
        }
      );

      solutions.push_back(wolves);
    }

    double elapsedTime = this->runTimer.getElapsedTime();
    this->runTimer.stop();

    this->currRunIterationNumber = 0;

    return GWOResult{
      solutions,
      inputBuildings,
      bestFitnesses,
      averageFitnesses,
      worstFitnesses,
      elapsedTime,
      numWolves,
      numIterations,
      alphaDecayRate
    };
  };

  int32_t GWO::getCurrentRunIterationNumber()
  {
    return this->currRunIterationNumber;
  }

  void GWO::computeWolfValues(
    eastl::vector<Solution>& wolves,
    eastl::vector<double>& wolfMutationRates,
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
    for (Solution& wolf : wolves) {
      wolf.setFitness(computeSolutionFitness(wolf,
                                             inputBuildings,
                                             boundingArea,
                                             flowRates,
                                             buildingDistanceWeight));
    }

    double maxFitness = eastl::max_element(
      wolves.begin(),
      wolves.end(),
      [](const Solution& solutionA, const Solution& solutionB) -> bool {
        return solutionA.getFitness() < solutionB.getFitness();
      })->getFitness();
    double minFitness = eastl::min_element(
      wolves.begin(),
      wolves.end(),
      [](const Solution& solutionA, const Solution& solutionB) -> bool {
        return solutionA.getFitness() < solutionB.getFitness();
      })->getFitness();
    double fitnessLength = maxFitness - minFitness;
    for (int32_t i = 0; i < wolves.size(); i++) {
      double wolfFitness = wolves[i].getFitness();
      wolfMutationRates[i] = 1 - ((maxFitness - wolfFitness) / fitnessLength);
    }
  }

  void GWO::updateWolves(
    eastl::vector<Solution>& wolves,
    const float& alpha,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    // Get best three wolves.
    // First element is the alpha wolf.
    // Second element is the beta wolf.
    // Third element is the delta wolf.
    eastl::array<Solution, 3> bestWolves{ wolves[0], wolves[1], wolves[2] };

    constexpr int32_t numLeadingWolves = 3;
    eastl::array<float, numLeadingWolves> A{ 0.f, 0.f, 0.f };
    eastl::array<float, numLeadingWolves> C{ 0.f, 0.f, 0.f };
    for (int32_t i = 0; i < numLeadingWolves; i++) {
      A[i] = (2 * alpha * cx::getRandomRealUniformly(0.f, 1.f)) - alpha;
      C[i] = 2 * cx::getRandomRealUniformly(0.f, 1.f);
    }

    float minX = boundingArea.vertices[0].x;
    float minY = boundingArea.vertices[0].y;
    float maxX = boundingArea.vertices[2].x;
    float maxY = boundingArea.vertices[2].y;
    float width = maxX - minX;
    float height = maxY - minY;

    cx::VecN alphaSolVecN = convertSolutionToVecN(
      translateSolutionOrigin(bestWolves[0], -minX, -minY));
    cx::VecN betaSolVecN = convertSolutionToVecN(
      translateSolutionOrigin(bestWolves[1], -minX, -minY));
    cx::VecN deltaSolVecN = convertSolutionToVecN(
      translateSolutionOrigin(bestWolves[2], -minX, -minY));

    std::cout << "#############\n";

    std::cout << "Bounding Area Dimensions:\t"
              << "W: " << (maxX - minX) << "\t"
              << "H: " << (maxY - minY) << "\n";

    std::cout << "Alpha\t|";
    printVecN(alphaSolVecN);

    std::cout << "Beta\t|";
    printVecN(betaSolVecN);

    std::cout << "Delta\t|";
    printVecN(deltaSolVecN);

    std::cout << "alpha\t| " << alpha << "\n";
    std::cout << "A\t| " << A[0] << "\t" << A[1] << "\t" << A[2] << "\n";
    std::cout << "C\t| " << C[0] << "\t" << C[1] << "\t" << C[2] << "\n";
    std::cout << "min\t| (" << minX << ", " << minY << ")\n";
    std::cout << "max\t| (" << maxX << ", " << maxY << ")\n";

    std::cout << "#############\n";

    for (Solution& wolf : wolves) {
      cx::VecN X = convertSolutionToVecN(
        translateSolutionOrigin(wolf, -minX, -minY));

      cx::VecN alphaD = cx::vecNAbs((C[0] * alphaSolVecN) - X);
      cx::VecN betaD = cx::vecNAbs((C[1] * betaSolVecN) - X);
      cx::VecN deltaD = cx::vecNAbs((C[2] * deltaSolVecN) - X);

      cx::VecN X1 = wrapAroundSolutionVecN(
        alphaSolVecN - A[0] * alphaD, 0.f, 0.f, width, height);
      cx::VecN X2 = wrapAroundSolutionVecN(
        betaSolVecN - A[1] * betaD, 0.f, 0.f, width, height);
      cx::VecN X3 = wrapAroundSolutionVecN(
        deltaSolVecN - A[2] * deltaD, 0.f, 0.f, width, height);

      // Translate buildings back to world origin.
      wolf = translateSolutionOrigin(
        convertVecNToSolution((X1 + X2 + X3) / 3), minX, minY);

      std::cout << "X\t| ";
      printVecN(X);

      std::cout << "aD\t| ";
      printVecN(alphaD);

      std::cout << "bD\t| ";
      printVecN(betaD);

      std::cout << "dD\t| ";
      printVecN(deltaD);

      std::cout << "cAAD| ";
      printVecN(A[0] * alphaD);

      std::cout << "cABD| ";
      printVecN(A[1] * betaD);

      std::cout << "cADD| ";
      printVecN(A[2] * deltaD);

      std::cout << "X1\t| ";
      printVecN(X1);

      std::cout << "Xn1\t| ";
      printVecN(alphaSolVecN - A[0] * alphaD);

      std::cout << "X2\t| ";
      printVecN(X2);

      std::cout << "Xn2\t| ";
      printVecN(betaSolVecN - A[1] * betaD);

      std::cout << "X3\t| ";
      printVecN(X3);

      std::cout << "Xn3\t| ";
      printVecN(deltaSolVecN - A[2] * deltaD);

      std::cout << "w\t| ";
      printVecN((X1 + X2 + X3) / 3);

      std::cout << "------------------\n";

      // Do a crossover for the building angles, since they get wrongly modified
      // by the mixing of the alpha, beta, and delta wolves.
      for (int32_t i = 0; i < wolf.getNumBuildings(); i++) {
        int32_t partnerIndex = cx::getRandomIntUniformly(0, 2);
        wolf.setBuildingAngle(i, bestWolves[partnerIndex].getBuildingAngle(i));
      }
    }
  }

  void GWO::mutateWolves(
    eastl::vector<Solution>& wolves,
    eastl::vector<double>& wolfMutationRates,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    for (int32_t i = 0; i < wolves.size(); i++) {
      Solution& wolf = wolves[i];
      double wolfMutationRate = wolfMutationRates[i];
      if (cx::getRandomRealUniformly(0.0, 1.0) <= wolfMutationRate) {
        this->mutateSolution(wolf, boundingArea, inputBuildings,
                             keepInfeasibleSolutions);
      }
    }
  }

  void GWO::mutateSolution(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    eastl::array<eastl::function<void(Solution&,
                                      const corex::core::NPolygon&,
                                      const eastl::vector<InputBuilding>&,
                                      const bool&)>,
      3> mutationFunctions = {
      [](Solution& solution,
         const corex::core::NPolygon& boundingArea,
         const eastl::vector<InputBuilding>& inputBuildings,
         const bool& keepInfeasibleSolutions)
      {
        applyBuddyBuddyOperator(solution, boundingArea,
                                inputBuildings, -1, -1,
                                keepInfeasibleSolutions);
      },
      [](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings,
             const bool& keepInfeasibleSolutions)
      {
        applyShakingOperator(solution, boundingArea,
                             inputBuildings, keepInfeasibleSolutions);
      },
      [](Solution& solution,
         const corex::core::NPolygon& boundingArea,
         const eastl::vector<InputBuilding>& inputBuildings,
         const bool& keepInfeasibleSolutions)
      {
        applyJiggleOperator(solution, boundingArea,
                            inputBuildings, keepInfeasibleSolutions);
      }
    };

    Solution tempSolution;
    do {
      tempSolution = solution;
      //const int32_t mutationFuncIndex = cx::getRandomIntUniformly(
      //  0, static_cast<int32_t>(mutationFunctions.size() - 1));
      const int32_t mutationFuncIndex = 0;
      mutationFunctions[mutationFuncIndex](tempSolution,
                                           boundingArea,
                                           inputBuildings,
                                           keepInfeasibleSolutions);
    } while (!keepInfeasibleSolutions
             && !isSolutionFeasible(tempSolution, boundingArea,
                                    inputBuildings));

    solution = tempSolution;
  }

  cx::VecN GWO::createRandomVector(const int32_t vectorSize)
  {
    cx::VecN randomVecN{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++) {
      randomVecN[i] = cx::getRandomRealUniformly(0.f, 1.f);
    }

    return randomVecN;
  }
}
