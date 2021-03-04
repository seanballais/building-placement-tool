#include <cassert>

#include <EASTL/array.h>
#include <EASTL/vector.h>

#include <corex/core/utils.hpp>
#include <corex/core/ds/NPolygon.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
#include <bpt/GWO.hpp>
#include <bpt/operators.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>
#include <bpt/ds/Result.hpp>

namespace bpt
{
  Result GWO::generateSolutions(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const int32_t numWolves,
    const int32_t numIterations,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<Solution> wolves;
    eastl::vector<float> wolfMutationRates;

    // Generate initial wolves.
    for (int32_t i = 0; i < numWolves; i++) {
      wolves.push_back(generateRandomSolution(inputBuildings,
                                              boundingArea));
    }

    for (int32_t i = 0; i < numIterations; i++) {
      // Evaluate individual fitness and mutation rate.
      // Update individuals based on a custom search operator.
      // Perform adaptive mutation.

      solutions.push_back(wolves);
    }
  };

  eastl::vector<Solution> GWO::crossoverSolutions(
    const CrossoverType& type,
    const Solution& solutionA,
    const Solution& solutionB,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const bool& keepInfeasibleSolutions)
  {
    assert(type != CrossoverType::NONE);

    switch (type) {
      case CrossoverType::UNIFORM:
        return performUniformCrossover(solutionA, solutionB,
                                       boundingArea, inputBuildings,
                                       keepInfeasibleSolutions);
      case CrossoverType::BOX:
        return performBoxCrossover(solutionA, solutionB,
                                   boundingArea, inputBuildings,
                                   keepInfeasibleSolutions);
      default:
        // TODO: Raise an error since we did not choose a crossover operator.
        break;
    }

    return {};
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
  }
}
