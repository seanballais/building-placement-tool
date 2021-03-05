#include <cmath>
#include <iostream>

#include <EASTL/vector.h>
#include <iprof/iprof.hpp>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/Timer.hpp>
#include <corex/core/utils.hpp>

#include <bpt/ds/InputBuilding.hpp>

#include <bpt/HC.hpp>
#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
#include <bpt/operators.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  eastl::vector<eastl::vector<Solution>> HC::generateSolution(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const double timeLimit)
  {
    Solution initialSolution = generateRandomSolution(inputBuildings,
                                                      boundingArea);
    initialSolution.setFitness(computeSolutionFitness(initialSolution,
                                                      inputBuildings,
                                                      boundingArea,
                                                      flowRates,
                                                      floodProneAreas,
                                                      landslideProneAreas,
                                                      floodProneAreaPenalty,
                                                      landslideProneAreaPenalty,
                                                      buildingDistanceWeight));
    return this->generateSolution(initialSolution,
                                  inputBuildings,
                                  boundingArea,
                                  flowRates,
                                  floodProneAreas,
                                  landslideProneAreas,
                                  floodProneAreaPenalty,
                                  landslideProneAreaPenalty,
                                  buildingDistanceWeight,
                                  timeLimit);
  }

  eastl::vector<eastl::vector<Solution>> HC::generateSolution(
    Solution initialSolution,
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const double timeLimit)
  {
    eastl::vector<eastl::vector<Solution>> solutions;
    Solution bestSolution = initialSolution;
    solutions.push_back({ bestSolution });

    cx::Timer timer;
    timer.start();
    while (timer.getElapsedTime() <= timeLimit) {
      std::cout << timer.getElapsedTime() << "\n";
      Solution candidateSolution = initialSolution;
      this->perturbSolution(candidateSolution, boundingArea, inputBuildings,
                            flowRates, floodProneAreas, landslideProneAreas,
                            floodProneAreaPenalty, landslideProneAreaPenalty,
                            buildingDistanceWeight);
      if (candidateSolution.getFitness() <= bestSolution.getFitness()) {
        initialSolution = candidateSolution;
        bestSolution = initialSolution;
        solutions.push_back({ bestSolution });
      }
    }

    timer.stop();

    return solutions;
  }

  void HC::perturbSolution(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
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
        applyBuddyBuddyOperator(solution, boundingArea,
                                inputBuildings, -1, -1,
                                keepInfeasibleSolutions);
      },
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings,
             const bool& keepInfeasibleSolutions)
      {
        applyShakingOperator(solution, boundingArea,
                             inputBuildings, keepInfeasibleSolutions);
      },
      [this](Solution& solution,
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
      const int32_t mutationFuncIndex = cx::getRandomIntUniformly(
        0, static_cast<int32_t>(mutationFunctions.size() - 1));
      mutationFunctions[mutationFuncIndex](tempSolution,
                                           boundingArea,
                                           inputBuildings,
                                           true);
    } while (!isSolutionFeasible(tempSolution, boundingArea, inputBuildings));
    solution = tempSolution;
    solution.setFitness(computeSolutionFitness(solution,
                                               inputBuildings,
                                               boundingArea,
                                               flowRates,
                                               floodProneAreas,
                                               landslideProneAreas,
                                               floodProneAreaPenalty,
                                               landslideProneAreaPenalty,
                                               buildingDistanceWeight));
  }
}
