#include <cassert>
#include <cmath>
#include <iostream>

#include <EASTL/algorithm.h>
#include <EASTL/array.h>
#include <EASTL/functional.h>
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
    , currRunIterationNumber(0) {}

  GWOResult GWO::generateSolutions(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const int32_t numWolves,
    const int32_t numIterations,
    const float cValue,
    const float alphaDecayRate,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const bool isLocalSearchEnabled,
    const double timeLimit,
    const bool &keepInfeasibleSolutions)
  {
    // TODO: Fix mutation.
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<Solution> wolves;
    eastl::vector<double> wolfMutationRates;
    eastl::vector<double> bestFitnesses;
    eastl::vector<double> averageFitnesses;
    eastl::vector<double> worstFitnesses;

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

    std::sort(
      wolves.begin(),
      wolves.end(),
      [](Solution& solutionA, Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );

    double worstFitness = wolves.back().getFitness();
    double bestFitness = wolves[0].getFitness();

    double averageFitness = 0.0;
    for (Solution& wolf : wolves) {
      averageFitness += wolf.getFitness();
    }

    averageFitness /= wolves.size();

    bestFitnesses.push_back(bestFitness);
    averageFitnesses.push_back(averageFitness);
    worstFitnesses.push_back(worstFitness);

    solutions.push_back(wolves);

    float alpha = 2.f;
    for (int32_t i = 0; i < numIterations; i++) {
      this->currRunIterationNumber = i;

      // Update individuals based on a custom search operator.
      this->updateWolves(
        wolves,
        alpha,
        cValue,
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

      // Sort GWO debugging data based on solution fitness.
      // Rank wolves pre-sorting. Ranking starts at 0.
      auto cmpFunc = [](Solution& a, Solution& b) -> bool {
        return a.getFitness() < b.getFitness();
      };
      eastl::vector<int32_t> dataIndices = cx::rankVectors(wolves, cmpFunc);

      // Sort the wolves now. We could do this earlier, but it looks cleaner
      // to just put it here.
      std::sort(
        wolves.begin(),
        wolves.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return solutionA.getFitness() < solutionB.getFitness();
        }
      );

      // Compute statistical data.
      worstFitness = wolves.back().getFitness();
      bestFitness = wolves[0].getFitness();

      averageFitness = 0.0;
      for (Solution& wolf : wolves) {
        averageFitness += wolf.getFitness();
      }

      averageFitness /= static_cast<float>(wolves.size());

      bestFitnesses.push_back(bestFitness);
      averageFitnesses.push_back(averageFitness);
      worstFitnesses.push_back(worstFitness);

      solutions.push_back(wolves);

      alpha = 2.f - (2.f * (static_cast<float>(i) / numIterations));
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
    const float cValue,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    constexpr int32_t numLeaders = 3;

    // Building x and y positions data.
    eastl::array<Solution, numLeaders> leadingWolves{
      wolves[0], wolves[1], wolves[2]
    };

    cx::VecN alphaVecN = convertSolutionToVecN(leadingWolves[0]);
    cx::VecN betaVecN = convertSolutionToVecN(leadingWolves[1]);
    cx::VecN deltaVecN = convertSolutionToVecN(leadingWolves[2]);

    // Coefficient values of the leading wolves.
    cx::VecN defVecN(inputBuildings.size() * 3, 0.f);
    eastl::array<cx::VecN, numLeaders> ALeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> CLeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r1Leaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r2Leaders{ defVecN, defVecN, defVecN };

    cx::Point minBoundingPt = boundingArea.vertices[0];

    float boundWidth = boundingArea.vertices[1].x
                       - boundingArea.vertices[0].x;
    float boundHeight = boundingArea.vertices[3].y
                        - boundingArea.vertices[0].y;

    for (Solution& wolf : wolves) {
      for (int32_t n = 0; n < numLeaders; n++) {
        r1Leaders[n] = this->createRandomVector(defVecN.size());
        r2Leaders[n] = this->createRandomVector(defVecN.size(), -1.f, 1.f);
        ALeaders[n] = this->createACoefficientVector(defVecN.size(),
                                                     r1Leaders[n],
                                                     alpha);
        CLeaders[n] = this->createCCoefficientVector(defVecN.size(),
                                                     cValue,
                                                     r2Leaders[n]);
      }

      // Compute the building x and y positions.
      cx::VecN wolfSol = convertSolutionToVecN(wolf);

      auto Da = cx::vecNAbs((CLeaders[0] + alphaVecN) - wolfSol);
      auto Db = cx::vecNAbs((CLeaders[1] + betaVecN) - wolfSol);
      auto Dd = cx::vecNAbs((CLeaders[2] + deltaVecN) - wolfSol);

      auto X1 = alphaVecN - cx::pairwiseMult(ALeaders[0], Da);
      auto X2 = betaVecN - cx::pairwiseMult(ALeaders[1], Db);
      auto X3 = deltaVecN - cx::pairwiseMult(ALeaders[2], Dd);

      wolfSol = (X1 + X2 + X3) / 3.f;

      // Add orientations to the wolf VecN, and clamp building positions to the
      // bounding area too.
      for (int32_t i = 0; i < inputBuildings.size(); i++) {
        // Crossover to get orientation.
        int32_t partnerIdx = cx::selectItemRandomly(eastl::vector{ 0, 1, 2 });
        wolfSol[(i * 3) + 2] = leadingWolves[partnerIdx].getBuildingAngle(i);

        cx::Rectangle buildingRect {
          wolf.getBuildingXPos(i),
          wolf.getBuildingYPos(i),
          inputBuildings[i].width,
          inputBuildings[i].length,
          wolf.getBuildingAngle(i)
        };

        cx::Polygon<4> poly = cx::rotateRectangle(buildingRect);

        float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
        float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

        float minBuildingXVal = minBoundingPt.x + (polyWidth / 2);
        float maxBuildingXVal = minBoundingPt.x
                                + (boundWidth - (polyWidth / 2));
        float minBuildingYVal = minBoundingPt.y + (polyHeight / 2);
        float maxBuildingYVal = minBoundingPt.y
                                + (boundHeight - (polyHeight / 2));

        if (minBuildingXVal > maxBuildingXVal) {
          eastl::swap(minBuildingXVal, maxBuildingXVal);
        }

        if (minBuildingYVal > maxBuildingYVal) {
          eastl::swap(minBuildingYVal, maxBuildingYVal);
        }

        wolfSol[i * 3] = cx::clamp(wolfSol[i * 3],
                                   minBuildingXVal,
                                   maxBuildingXVal);
        wolfSol[(i * 3) + 1] = cx::clamp(wolfSol[(i * 3) + 1],
                                         minBuildingYVal,
                                         maxBuildingYVal);
      }

      wolf = convertVecNToSolution(wolfSol);
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
      4> mutationFunctions = {
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
      },
      [](Solution& solution,
         const corex::core::NPolygon& boundingArea,
         const eastl::vector<InputBuilding>& inputBuildings,
         const bool& keepInfeasibleSolutions)
      {
        applyOrientationFlipping(solution, boundingArea,
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

  cx::VecN GWO::createRandomVector(const int32_t vectorSize,
                                   float min, float max)
  {
    cx::VecN randomVecN{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++) {
      randomVecN[i] = cx::getRandomRealUniformly(min, max);
    }

    return randomVecN;
  }

  cx::VecN GWO::createACoefficientVector(const int32_t vectorSize,
                                         const cx::VecN randomVector1,
                                         const float alpha)
  {
    cx::VecN aCoefficient{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++) {
      aCoefficient[i] = (2 * alpha * randomVector1[i]) - alpha;
    }

    return aCoefficient;
  }

  cx::VecN GWO::createCCoefficientVector(const int32_t vectorSize,
                                         const float cValue,
                                         const cx::VecN randomVector2)
  {
    cx::VecN cCoefficient{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++ ) {
      cCoefficient[i] = cValue * randomVector2[i];
    }

    return cCoefficient;
  }

  cx::VecN GWO::getNormalizedSolutionOrientations(const Solution& solution)
  {
    cx::VecN orientations{ solution.getNumBuildings() };
    for (int32_t i = 0; i < orientations.size(); i++) {
      // Note: A building's angle can only either be 0° or 90°.
      orientations[i] = solution.getBuildingAngle(i) / 90.f;
    }

    return orientations;
  }

  cx::VecN GWO::getVecNItemMultiples(const cx::VecN& p,
                                     int32_t groupSize,
                                     int32_t ordinality,
                                     int32_t numItems)
  {
    // p's size must be divisible by the group size.
    assert(p.size() % groupSize == 0);

    cx::VecN collectedItems{ numItems };
    for (int32_t i = 0; i < numItems; i++) {
      collectedItems[i] = p[(i * groupSize) + ordinality];
    }

    return collectedItems;
  }

  cx::VecN GWO::applySigmoid(const cx::VecN& A, const cx::VecN& D)
  {
    cx::VecN s{ static_cast<int32_t>(A.size()) };
    for (int32_t i = 0; i < s.size(); i++) {
      s[i] = 1.f / (1.f + std::exp(-10 * ((A[i] * D[i]) - 0.5f)));
    }

    return s;
  }

  cx::VecN GWO::getBStep(const cx::VecN& s)
  {
    cx::VecN bStep{ static_cast<int32_t>(s.size()) };
    for (int32_t i = 0; i < bStep.size(); i++) {
      float randn = cx::getRandomRealUniformly(0.f, 1.f);
      bStep[i] = cx::floatGreEqual(s[i], randn);
    }

    return bStep;
  }

  cx::VecN GWO::getBinX(const cx::VecN& X, const cx::VecN& bStep)
  {
    assert(X.size() == bStep.size());

    cx::VecN XVecN{ static_cast<int32_t>(X.size()) };
    for (int32_t i = 0; i < XVecN.size(); i++) {
      XVecN[i] = cx::floatGreEqual(X[i] + bStep[i], 1.f);
    }

    return XVecN;
  };
}
