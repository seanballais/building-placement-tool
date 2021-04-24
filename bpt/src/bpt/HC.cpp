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
  HC::HC()
    : currRunIterationNumber(0) {}

  eastl::vector<eastl::vector<Solution>>
  HC::generateSolution(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty, const float buildingDistanceWeight,
    const int32_t numIters,
    int32_t* const currIterNumberPtr)
  {
    Solution initialSolution = generateRandomSolution(inputBuildings,
                                                      boundingArea);
    initialSolution.setFitness(computeSolutionFitness(initialSolution,
                                                      inputBuildings,
                                                      boundingArea,
                                                      flowRates,
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
                                  numIters,
                                  currIterNumberPtr);
  }

  eastl::vector<eastl::vector<Solution>>
  HC::generateSolution(
    Solution initialSolution,
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const int32_t numIters,
    int32_t* const currIterNumberPtr)
  {
    eastl::vector<eastl::vector<Solution>> solutions;
    Solution bestSolution = initialSolution;
    solutions.push_back({ bestSolution });

    for (int32_t i = 0; i < numIters; i++) {
      (*currIterNumberPtr)++;

      Solution candidateSolution = initialSolution;
      this->perturbSolution(candidateSolution,
                            boundingArea,
                            inputBuildings,
                            flowRates,
                            floodProneAreas,
                            landslideProneAreas,
                            floodProneAreaPenalty,
                            landslideProneAreaPenalty,
                            buildingDistanceWeight);
      if (candidateSolution.getFitness() <= bestSolution.getFitness()) {
        initialSolution = candidateSolution;
        bestSolution = initialSolution;
      }

      solutions.push_back({ bestSolution });

      this->currRunIterationNumber++;
    }

    this->currRunIterationNumber = 0;

    return solutions;
  }

  int32_t HC::getCurrentRunIterationNumber() const
  {
    return this->currRunIterationNumber;
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

    const int32_t mutationFuncIndex = cx::getRandomIntUniformly(
      0, static_cast<int32_t>(mutationFunctions.size() - 1));
    mutationFunctions[mutationFuncIndex](solution,
                                         boundingArea,
                                         inputBuildings,
                                         true);
    solution.setFitness(computeSolutionFitness(solution,
                                               inputBuildings,
                                               boundingArea,
                                               flowRates,
                                               buildingDistanceWeight));
  }
}
