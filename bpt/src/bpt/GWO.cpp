#include <cassert>
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
    , currRunIterationNumber(0)
    , hillClimbing()
    , recentRunData() {}

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

    // Clear previous run data.
    this->recentRunData.alphaWolves.clear();
    this->recentRunData.betaWolves.clear();
    this->recentRunData.deltaWolves.clear();
    this->recentRunData.r1Alphas.clear();
    this->recentRunData.r1Betas.clear();
    this->recentRunData.r1Deltas.clear();
    this->recentRunData.r2Alphas.clear();
    this->recentRunData.r2Betas.clear();
    this->recentRunData.r2Deltas.clear();
    this->recentRunData.Aalphas.clear();
    this->recentRunData.Abetas.clear();
    this->recentRunData.Adeltas.clear();
    this->recentRunData.Calphas.clear();
    this->recentRunData.Cbetas.clear();
    this->recentRunData.Cdeltas.clear();
    this->recentRunData.Dalphas.clear();
    this->recentRunData.Dbetas.clear();
    this->recentRunData.Ddeltas.clear();
    this->recentRunData.X1s.clear();
    this->recentRunData.X2s.clear();
    this->recentRunData.X3s.clear();
    this->recentRunData.oldWolves.clear();
    this->recentRunData.newWolves.clear();

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

      this->mutateWolves(
        wolves,
        wolfMutationRates,
        boundingArea,
        inputBuildings,
        keepInfeasibleSolutions);

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

      cx::reorderVector(this->recentRunData.r1Alphas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.r1Betas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.r1Deltas.back(), dataIndices);

      cx::reorderVector(this->recentRunData.r2Alphas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.r2Betas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.r2Deltas.back(), dataIndices);

      cx::reorderVector(this->recentRunData.Aalphas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Abetas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Adeltas.back(), dataIndices);

      cx::reorderVector(this->recentRunData.Calphas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Cbetas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Cdeltas.back(), dataIndices);

      cx::reorderVector(this->recentRunData.Dalphas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Dbetas.back(), dataIndices);
      cx::reorderVector(this->recentRunData.Ddeltas.back(), dataIndices);

      cx::reorderVector(this->recentRunData.X1s.back(), dataIndices);
      cx::reorderVector(this->recentRunData.X2s.back(), dataIndices);
      cx::reorderVector(this->recentRunData.X3s.back(), dataIndices);

      cx::reorderVector(this->recentRunData.oldWolves.back(), dataIndices);
      cx::reorderVector(this->recentRunData.newWolves.back(), dataIndices);

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

  const GWOData& GWO::getRecentRunData()
  {
    return this->recentRunData;
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
    eastl::array<Solution, 3> leadingWolves{ wolves[0], wolves[1], wolves[2] };

    // Get the delta vector.
    cx::Point minBoundingPt = boundingArea.vertices[0];

    float boundWidth = boundingArea.vertices[1].x
                       - boundingArea.vertices[0].x;
    float boundHeight = boundingArea.vertices[3].y
                        - boundingArea.vertices[0].y;

    cx::VecN deltaVec(inputBuildings.size() * 3);
    cx::VecN newOrigDelta(inputBuildings.size() * 3);
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      deltaVec[i * 3] = -minBoundingPt.x;
      deltaVec[(i * 3) + 1] = -minBoundingPt.y;

      newOrigDelta[i * 3] = -boundWidth / 2.f;
      newOrigDelta[(i * 3) + 1] = -boundHeight / 2.f;
    }

    cx::VecN alphaVecN = convertSolutionToVecN(leadingWolves[0]);
    cx::VecN betaVecN = convertSolutionToVecN(leadingWolves[1]);
    cx::VecN deltaVecN = convertSolutionToVecN(leadingWolves[2]);

    // Translated top three wolves.
    alphaVecN = cx::translateVecN(alphaVecN, deltaVec);
    betaVecN = cx::translateVecN(betaVecN, deltaVec);
    deltaVecN = cx::translateVecN(deltaVecN, deltaVec);

    // Translate the wolves to put their origins in the middle of the bounding
    // area.
    alphaVecN = cx::translateVecN(alphaVecN, newOrigDelta);
    betaVecN = cx::translateVecN(betaVecN, newOrigDelta);
    deltaVecN = cx::translateVecN(deltaVecN, newOrigDelta);

    this->recentRunData.alphaWolves.push_back(alphaVecN);
    this->recentRunData.betaWolves.push_back(betaVecN);
    this->recentRunData.deltaWolves.push_back(deltaVecN);
    this->recentRunData.alphaValues.push_back(alpha);

    // Coefficient values of the leading wolves.
    constexpr int32_t numLeaders = 3;
    cx::VecN defVecN(inputBuildings.size() * 3, 0.f);
    eastl::array<cx::VecN, numLeaders> ALeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> CLeaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r1Leaders{ defVecN, defVecN, defVecN };
    eastl::array<cx::VecN, numLeaders> r2Leaders{ defVecN, defVecN, defVecN };

    // For the GWO debugging data.
    eastl::vector<cx::VecN> r1Alphas;
    eastl::vector<cx::VecN> r1Betas;
    eastl::vector<cx::VecN> r1Deltas;

    eastl::vector<cx::VecN> r2Alphas;
    eastl::vector<cx::VecN> r2Betas;
    eastl::vector<cx::VecN> r2Deltas;

    eastl::vector<cx::VecN> Aalphas;
    eastl::vector<cx::VecN> Abetas;
    eastl::vector<cx::VecN> Adeltas;

    eastl::vector<cx::VecN> Calphas;
    eastl::vector<cx::VecN> Cbetas;
    eastl::vector<cx::VecN> Cdeltas;

    eastl::vector<cx::VecN> Dalphas;
    eastl::vector<cx::VecN> Dbetas;
    eastl::vector<cx::VecN> Ddeltas;

    eastl::vector<cx::VecN> X1s;
    eastl::vector<cx::VecN> X2s;
    eastl::vector<cx::VecN> X3s;

    eastl::vector<cx::VecN> oldWolves;
    eastl::vector<cx::VecN> newWolves;

    for (Solution& wolf : wolves) {
      for (int32_t n = 0; n < numLeaders; n++) {
        r1Leaders[n] = this->createRandomVector(defVecN.size());
        r2Leaders[n] = this->createRandomVector(defVecN.size());
        ALeaders[n] = this->createACoefficientVector(defVecN.size(),
                                                     r1Leaders[n],
                                                     alpha);
        CLeaders[n] = this->createCCoefficientVector(defVecN.size(),
                                                     r2Leaders[n]);
      }

      r1Alphas.push_back(r1Leaders[0]);
      r1Betas.push_back(r1Leaders[1]);
      r1Deltas.push_back(r1Leaders[2]);

      r2Alphas.push_back(r2Leaders[0]);
      r2Betas.push_back(r2Leaders[1]);
      r2Deltas.push_back(r2Leaders[2]);

      Aalphas.push_back(ALeaders[0]);
      Abetas.push_back(ALeaders[1]);
      Adeltas.push_back(ALeaders[2]);

      Calphas.push_back(CLeaders[0]);
      Cbetas.push_back(CLeaders[1]);
      Cdeltas.push_back(CLeaders[2]);

      cx::VecN wolfSol = convertSolutionToVecN(wolf);
      wolfSol = cx::translateVecN(wolfSol, deltaVec);

      // Translate the wolf to move the origin in the middle of the bounding
      // area.
      wolfSol = cx::translateVecN(wolfSol, newOrigDelta);

      auto Da = cx::vecNAbs(cx::pairwiseMult(CLeaders[0], alphaVecN) - wolfSol);
      auto Db = cx::vecNAbs(cx::pairwiseMult(CLeaders[1], betaVecN) - wolfSol);
      auto Dd = cx::vecNAbs(cx::pairwiseMult(CLeaders[2], deltaVecN) - wolfSol);

      auto X1 = alphaVecN - cx::pairwiseMult(ALeaders[0], Da);
      auto X2 = betaVecN - cx::pairwiseMult(ALeaders[1], Db);
      auto X3 = deltaVecN - cx::pairwiseMult(ALeaders[2], Dd);

      Dalphas.push_back(Da);
      Dbetas.push_back(Db);
      Ddeltas.push_back(Dd);

      X1s.push_back(X1);
      X2s.push_back(X2);
      X3s.push_back(X3);

      oldWolves.push_back(wolfSol);

      wolfSol = (X1 + X2 + X3) / 3.f;

      // Do a crossover for the building angles, since they get wrongly modified
      // by the mixing of the alpha, beta, and delta wolves. Clamp building
      // positions to the bounding area too.
      for (int32_t i = 0; i < inputBuildings.size(); i++) {
        int32_t partnerIndex = cx::getRandomIntUniformly(0, 2);
        Solution partnerWolf = leadingWolves[partnerIndex];
        wolfSol[(i * 3) + 2] = partnerWolf.getBuildingAngle(i);

        // Clamping time.
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

        // We need a buffer since we do not buildings to be intersecting by
        // exactly 1 pixel. Note also that we are in origin at this point.
        float minBuildingXVal = (polyWidth / 2) + 1;
        float maxBuildingXVal = (boundWidth - (polyWidth / 2)) - 1;
        float minBuildingYVal = (polyHeight / 2) + 1;
        float maxBuildingYVal = (boundHeight - (polyHeight / 2)) - 1;

        if (minBuildingXVal > maxBuildingXVal) {
          eastl::swap(minBuildingXVal, maxBuildingXVal);
        }

        if (minBuildingYVal > maxBuildingYVal) {
          eastl::swap(minBuildingYVal, maxBuildingYVal);
        }

//        wolfSol[i * 3] = cx::clamp(wolfSol[i * 3],
//                                   minBuildingXVal,
//                                   maxBuildingXVal);
//        wolfSol[(i * 3) + 1] = cx::clamp(wolfSol[(i * 3) + 1],
//                                         minBuildingYVal,
//                                         maxBuildingYVal);
      }

      newWolves.push_back(wolfSol);

      // Translate the buildings to put the origin in the top left corner of
      // the bounding area.
      wolfSol = cx::translateVecN(wolfSol, -newOrigDelta);

      // Translate the buildings pack to their original position.
      wolfSol = cx::translateVecN(wolfSol, -deltaVec);

      wolf = convertVecNToSolution(wolfSol);
    }

    this->recentRunData.r1Alphas.push_back(r1Alphas);
    this->recentRunData.r1Betas.push_back(r1Betas);
    this->recentRunData.r1Deltas.push_back(r1Deltas);

    this->recentRunData.r2Alphas.push_back(r2Alphas);
    this->recentRunData.r2Betas.push_back(r2Betas);
    this->recentRunData.r2Deltas.push_back(r2Deltas);

    this->recentRunData.Aalphas.push_back(Aalphas);
    this->recentRunData.Abetas.push_back(Abetas);
    this->recentRunData.Adeltas.push_back(Adeltas);

    this->recentRunData.Calphas.push_back(Calphas);
    this->recentRunData.Cbetas.push_back(Cbetas);
    this->recentRunData.Cdeltas.push_back(Cdeltas);

    this->recentRunData.Dalphas.push_back(Dalphas);
    this->recentRunData.Dbetas.push_back(Dbetas);
    this->recentRunData.Ddeltas.push_back(Ddeltas);

    this->recentRunData.X1s.push_back(X1s);
    this->recentRunData.X2s.push_back(X2s);
    this->recentRunData.X3s.push_back(X3s);

    this->recentRunData.oldWolves.push_back(oldWolves);
    this->recentRunData.newWolves.push_back(newWolves);
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
      const int32_t mutationFuncIndex = 1;
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
      cCoefficient[i] = 2 * randomVector2[i];
    }

    return cCoefficient;
  }
}
