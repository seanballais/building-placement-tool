#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>

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
#include <bpt/operators.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/GWOResult.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>
#include <bpt/ds/Wolf.hpp>

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
    const float fMin,
    const float fMax,
    const float CR,
    const float epsilon,
    const bool &keepInfeasibleSolutions)
  {
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<Wolf> wolves;
    eastl::vector<double> bestFitnesses;
    eastl::vector<double> averageFitnesses;
    eastl::vector<double> worstFitnesses;

    this->runTimer.start();

    // Generate initial wolves.
    for (int32_t i = 0; i < numWolves; i++) {
      std::cout << "Generating Wolf #" << i << "\n";
      Solution randomSolution = generateRandomSolution(inputBuildings,
                                                       boundingArea);
      randomSolution.setFitness(
        computeSolutionFitness(randomSolution,
                               inputBuildings,
                               boundingArea,
                               flowRates,
                               buildingDistanceWeight)
      );

      wolves.push_back(Wolf{
        randomSolution,
        randomSolution
      });
    }

    std::sort(
      wolves.begin(),
      wolves.end(),
      [](const Wolf& a, const Wolf& b) {
        return a.currSolution.getFitness() < b.currSolution.getFitness();
      }
    );

    double worstFitness = wolves.back().currSolution.getFitness();
    double bestFitness = wolves[0].currSolution.getFitness();

    double averageFitness = 0.0;
    for (const Wolf& wolf : wolves) {
      averageFitness += wolf.currSolution.getFitness();
    }

    averageFitness /= wolves.size();

    bestFitnesses.push_back(bestFitness);
    averageFitnesses.push_back(averageFitness);
    worstFitnesses.push_back(worstFitness);

    eastl::vector<Solution> currIterWolves;
    std::transform(wolves.begin(), wolves.end(),
                   std::back_inserter(currIterWolves),
                   [](const Wolf& w) -> Solution { return w.currSolution; });
    solutions.push_back(currIterWolves);

    float alpha = 2.f;
    for (int32_t i = 0; i < numIterations; i++) {
      this->currRunIterationNumber = i;

      const Wolf& alphaWolf = wolves[0];
      const Wolf& betaWolf = wolves[1];
      const Wolf& deltaWolf = wolves[2];

      // Update individuals based on a custom search operator.
      eastl::vector<Wolf> newWolves = this->updateWolves(
        wolves,
        alphaWolf,
        betaWolf,
        deltaWolf,
        alpha,
        boundingArea,
        inputBuildings,
        keepInfeasibleSolutions);

      // Evaluate individual fitness and mutation rate.
      this->computeWolfValues(
        wolves,
        inputBuildings,
        boundingArea,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight);

      this->evolv

      this->computeWolfValues(
        wolves,
        inputBuildings,
        boundingArea,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight);

      // Selection.
      for (Wolf& wolf : wolves) {
        if (wolf.currSolution.getFitness() > wolf.bestSolution.getFitness()) {
          wolf.currSolution = wolf.bestSolution;
        } else {
          wolf.bestSolution = wolf.currSolution;
        }
      }

      // Sort the wolves now. We could do this earlier, but it looks cleaner
      // to just put it here.
      std::sort(
        wolves.begin(),
        wolves.end(),
        [](const Wolf& a, const Wolf& b) {
          return a.currSolution.getFitness() < b.currSolution.getFitness();
        }
      );

      // Compute statistical data.
      worstFitness = wolves.back().currSolution.getFitness();
      bestFitness = wolves[0].currSolution.getFitness();

      averageFitness = 0.0;
      for (const Wolf& wolf : wolves) {
        averageFitness += wolf.currSolution.getFitness();
      }

      averageFitness /= static_cast<float>(wolves.size());

      bestFitnesses.push_back(bestFitness);
      averageFitnesses.push_back(averageFitness);
      worstFitnesses.push_back(worstFitness);

      // Add in the wolves into the solutions collections.
      currIterWolves.clear();
      std::transform(wolves.begin(), wolves.end(),
                     std::back_inserter(currIterWolves),
                     [](const Wolf& w) -> Solution { return w.currSolution; });
      solutions.push_back(currIterWolves);

      float mu = 1.3f;
      float p = 6.f;

      alpha = 2.f
              - std::pow(
                  std::log(
                    1.f + (mu * std::pow(std::tan(i / numIterations), 3.f))
                  ),
                  p);
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
    eastl::vector<Wolf>& wolves,
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
    for (Wolf& wolf : wolves) {
      wolf.currSolution.setFitness(
        computeSolutionFitness(wolf.currSolution,
        inputBuildings,
        boundingArea,
        flowRates,
        buildingDistanceWeight)
      );
    }
  }

  eastl::vector<Wolf> GWO::updateWolves(
    const eastl::vector<Wolf> wolves,
    const Wolf& alphaWolf,
    const Wolf& betaWolf,
    const Wolf& deltaWolf,
    const float& alpha,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    eastl::vector<Wolf> newWolves = wolves;
    int32_t wolfIdx = 0;
    for (Wolf& wolf : newWolves) {
      wolf = this->huntPrey(wolfIdx, wolves,
                            alphaWolf, betaWolf, deltaWolf,
                            alpha, boundingArea, inputBuildings,
                            keepInfeasibleSolutions);

      wolfIdx++;
    }

    return newWolves;
  }

  eastl::vector<Wolf> GWO::evolveWolves(
    const eastl::vector<Wolf> wolves,
    const Wolf& alphaWolf,
    const Wolf& betaWolf,
    const Wolf& deltaWolf,
    const int32_t iter,
    const int32_t maxIters,
    const float& alpha,
    const float& fMin,
    const float& fMax,
    const float& CR,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    eastl::vector<Wolf> newWolves = wolves;
    int32_t wolfIdx = 0;
    for (Wolf& wolf : newWolves) {
      wolf = this->evolveWolf(wolfIdx, wolves, alphaWolf, betaWolf, deltaWolf,
                              iter, maxIters, alpha, fMin, fMax, CR,
                              boundingArea, inputBuildings);
      wolfIdx++;
    }

    return newWolves;
  }

  Wolf GWO::evolveWolf(
    const int32_t& wolfIdx,
    const eastl::vector<Wolf>& wolves,
    const Wolf& alphaWolf,
    const Wolf& betaWolf,
    const Wolf& deltaWolf,
    const int32_t iter,
    const int32_t maxIters,
    const float& alpha,
    const float& fMin,
    const float& fMax,
    const float& CR,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    // Mutation
    float F = fMin + (fMax - fMin)
              * (static_cast<float>(maxIters - (iter - 1))
                 / static_cast<float>(maxIters));

    std::vector<cx::VecN> leadingWolves{
      convertSolutionToVecN(alphaWolf.currSolution),
      convertSolutionToVecN(betaWolf.currSolution),
      convertSolutionToVecN(deltaWolf.currSolution)
    };

    cx::VecN V = leadingWolves[0] + F * (leadingWolves[1] - leadingWolves[2]);
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      V[(i * 3) + 2] = cx::selectItemRandomly(
        eastl::vector<float>{ 0.f, 90.f });
    }

    // Crossover
    cx::VecN targetWolf = convertSolutionToVecN(wolves[wolfIdx].currSolution);
    cx::VecN U = V;
    for (int32_t i = 0; i < U.size(); i++) {
      float probability = cx::getRandomRealUniformly(0.f, 1.f);
      int32_t randIndex = cx::getRandomIntUniformly(
        0, static_cast<int32_t>(U.size()) - 1);
      if (probability >= CR && i != randIndex) {
        U[i] = targetWolf[i];
      }
    }

    Wolf wolf = wolves[wolfIdx];
    wolf.currSolution = convertVecNToSolution(U);

    // Note: Selection is deferred to another part.

    return wolf;
  }

  Wolf GWO::huntPrey(
    const int32_t& wolfIdx,
    const eastl::vector<Wolf>& wolves,
    const Wolf& alphaWolf,
    const Wolf& betaWolf,
    const Wolf& deltaWolf,
    const float& alpha,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    eastl::vector<Solution> leadingWolves{
      alphaWolf.currSolution,
      betaWolf.currSolution,
      deltaWolf.currSolution
    };

    cx::VecN alphaVecN = convertSolutionToVecN(leadingWolves[0]);
    cx::VecN betaVecN  = convertSolutionToVecN(leadingWolves[1]);
    cx::VecN deltaVecN = convertSolutionToVecN(leadingWolves[2]);

    // Coefficient values of the leading wolves.
    constexpr int32_t numLeaders = 3;
    cx::VecN defVecN(inputBuildings.size() * 3, 0.f);
    eastl::array<cx::VecN, numLeaders> ALeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> CLeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r1Leaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r2Leaders{ defVecN, defVecN, defVecN };

    Wolf wolf = wolves[wolfIdx];

    for (int32_t n = 0; n < numLeaders; n++) {
      r1Leaders[n] = this->createRandomVector(defVecN.size());
      r2Leaders[n] = this->createRandomVector(defVecN.size(), -1.f, 1.f);
      ALeaders[n] = this->createACoefficientVector(defVecN.size(),
                                                   r1Leaders[n],
                                                   alpha);
      CLeaders[n] = this->createCCoefficientVector(defVecN.size(),
                                                   r2Leaders[n]);
    }

    // We could have used the current solution of a wolf here, but mGWO uses
    // the personal best solution of a wolf. Thus, we use the best solution
    // attribute.
    cx::VecN wolfSol = convertSolutionToVecN(wolf.bestSolution);

    auto Da = cx::vecNAbs((CLeaders[0] + alphaVecN) - wolfSol);
    auto Db = cx::vecNAbs((CLeaders[1] + betaVecN) - wolfSol);
    auto Dd = cx::vecNAbs((CLeaders[2] + deltaVecN) - wolfSol);

    auto X1 = alphaVecN - cx::pairwiseMult(ALeaders[0], Da);
    auto X2 = betaVecN - cx::pairwiseMult(ALeaders[1], Db);
    auto X3 = deltaVecN - cx::pairwiseMult(ALeaders[2], Dd);

    wolfSol = (X1 + X2 + X3) / 3.f;

    // Do a crossover for the building angles, since they get wrongly modified
    // by the mixing of the alpha, beta, and delta wolves.
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      int32_t partnerIndex = cx::getRandomIntUniformly(0, 2);
      Solution partnerWolf = leadingWolves[partnerIndex];
      wolfSol[(i * 3) + 2] = partnerWolf.getBuildingAngle(i);
    }

    wolf.currSolution = convertVecNToSolution(wolfSol);

    return wolf;
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
                                         const cx::VecN randomVector2)
  {
    cx::VecN cCoefficient{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++) {
      cCoefficient[i] = 4 * randomVector2[i];
    }

    return cCoefficient;
  }
}
