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
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>
#include <bpt/ds/Result.hpp>

namespace bpt
{
  GWO::GWO()
    : runTimer()
    , currRunIterationNumber(0)
    , hillClimbing() {}

  Result
  GWO::generateSolutions(
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

    solutions.push_back(wolves);

    cx::VecN alpha{static_cast<int32_t>(inputBuildings.size() * 3), 2.f};
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
      for (int32_t j = 0; j < alpha.size(); j++) {
        alpha[j] = cx::clamp(alpha[j] - alphaDecayRate, 0.f, 2.f);
      }

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

      solutions.push_back(wolves);
    }

    if (isLocalSearchEnabled) {
      Solution& bestSolution = wolves[0];
      auto lsGeneratedSolutions = this->hillClimbing.generateSolution(
        bestSolution,
        inputBuildings,
        boundingArea,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight,
        timeLimit);

      for (const eastl::vector<Solution>& solution : lsGeneratedSolutions) {
        bestFitnesses.push_back(solution[0].getFitness());
      }

      solutions.insert(solutions.end(),
                       lsGeneratedSolutions.begin(),
                       lsGeneratedSolutions.end());
    }

    // Sort solutions.


    Result runResult {
      solutions,
      bestFitnesses,
      averageFitnesses,
      worstFitnesses,
      this->runTimer.getElapsedTime()
    };

    this->runTimer.stop();

    return runResult;
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
                                             floodProneAreas,
                                             landslideProneAreas,
                                             floodProneAreaPenalty,
                                             landslideProneAreaPenalty,
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
    const cx::VecN& alpha,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    // Get best three wolves.
    // First element is the alpha wolf.
    // Second element is the beta wolf.
    // Third element is the delta wolf.
    eastl::array<Solution, 3> bestWolves;
    eastl::array<bool, 3> bestWolfHasBeenAssigned{ false, false, false };

    for (Solution& wolf : wolves) {
      if (!bestWolfHasBeenAssigned[0]
          || wolf.getFitness() < bestWolves[0].getFitness()) {
        bestWolves[2] = bestWolves[1];
        bestWolves[1] = bestWolves[0];
        bestWolves[0] = wolf;
        bestWolfHasBeenAssigned[0] = true;
      } else if (!bestWolfHasBeenAssigned[1]
                 || wolf.getFitness() < bestWolves[1].getFitness()) {
        bestWolves[2] = bestWolves[1];
        bestWolves[1] = wolf;
        bestWolfHasBeenAssigned[1] = true;
      } else if (!bestWolfHasBeenAssigned[2]
                 || wolf.getFitness() < bestWolves[2].getFitness()) {
        bestWolves[2] = wolf;
        bestWolfHasBeenAssigned[2] = true;
      }
    }

    int32_t coefficientSize = bestWolves[0].getNumBuildings() * 3;
    cx::VecN r1 = this->createRandomVector(coefficientSize);
    cx::VecN r2 = this->createRandomVector(coefficientSize);
    cx::VecN A = cx::multiplyTwoVecN((2 * alpha), r1) - alpha;
    cx::VecN C = 2 * r2;

    cx::VecN alphaSolVecN = convertSolutionToVecN(bestWolves[0]);
    cx::VecN betaSolVecN = convertSolutionToVecN(bestWolves[1]);
    cx::VecN deltaSolVecN = convertSolutionToVecN(bestWolves[2]);

    std::cout << "#############\n";

    std::cout << "Alpha\t|";
    printVecN(alphaSolVecN);

    std::cout << "Beta\t|";
    printVecN(betaSolVecN);

    std::cout << "Delta\t|";
    printVecN(deltaSolVecN);

    std::cout << "alpha\t|";
    printVecN(alpha);

    std::cout << "A\t|";
    printVecN(A);

    std::cout << "C\t|";
    printVecN(C);

    std::cout << "#############\n";

    float minX = boundingArea.vertices[0].x;
    float minY = boundingArea.vertices[0].y;
    float maxX = boundingArea.vertices[2].x;
    float maxY = boundingArea.vertices[2].y;

    // Breeding time, boys and girls!
    for (Solution& wolf : wolves) {
      cx::VecN X = convertSolutionToVecN(wolf);

      cx::VecN alphaD = cx::vecNAbs(alphaSolVecN - X);
      cx::VecN betaD = cx::vecNAbs(betaSolVecN - X);
      cx::VecN deltaD = cx::vecNAbs(deltaSolVecN - X);

      cx::VecN X1 = alphaSolVecN - cx::multiplyTwoVecN(A, alphaD);
      cx::VecN X2 = betaSolVecN - cx::multiplyTwoVecN(A, betaD);
      cx::VecN X3 = deltaSolVecN - cx::multiplyTwoVecN(A, deltaD);

      wolf = convertVecNToSolution((X1 + X2 + X3) / 3);

      std::cout << "X\t| ";
      printVecN(X);

      std::cout << "aD\t| ";
      printVecN(alphaD);

      std::cout << "bD\t| ";
      printVecN(betaD);

      std::cout << "dD\t| ";
      printVecN(deltaD);

      std::cout << "X1\t| ";
      printVecN(X1);

      std::cout << "X2\t| ";
      printVecN(X2);

      std::cout << "X3\t| ";
      printVecN(X3);

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
