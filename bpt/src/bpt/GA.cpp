#include <algorithm>
#include <cstdlib>
#include <random>

#include <EASTL/array.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

#include <bpt/GA.hpp>

namespace bpt
{
  GA::GA() {}

  Solution GA::generateSolution(eastl::vector<InputBuilding>& inputBuildings,
                                corex::core::NPolygon& boundingArea,
                                float mutationRate,
                                int32_t populationSize,
                                int32_t numGenerations)
  {
    eastl::array<Solution, populationSize> population;
    eastl::array<double, populationSize> populationFitnessValues;

    for (int32_t i = 0; i < populationSize; i++) {
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea);
      populationFitnessValues[i] = this->getSolutionFitness(population[i]);
    }

    std::default_random_engine randGenerator;
    std::uniform_int_distribution<int32_t> chromosomeDistribution{
      0, populationSize - 1
    };

    for (int32_t i = 0; i < numGenerations; i++) {
      // Selection
      int32_t numOffsprings = 0;
      eastl::array<Solution, populationSize> newPopulation;
      while (numOffsprings < populationSize) {
        int32_t parentAIndex = chromosomeDistribution(randGenerator);
        int32_t parentBIndex = 0;
        do {
          parentBIndex = chromosomeDistribution(randGenerator);
        } while (parentBIndex == parentAIndex);
      }

      // Crossover

      // Mutation
    }
  }

  Solution
  GA::generateRandomSolution(eastl::vector<InputBuilding>& inputBuildings,
                             corex::core::NPolygon& boundingArea)
  {
    float minX = *std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.x < ptB.x;
      }
    );
    float maxX = *std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.x > ptB.x;
      }
    );
    float minY = *std::min_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.y < ptB.y;
      }
    );
    float maxY = *std::max_element(
      boundingArea.vertices.begin(),
      boundingArea.vertices.end(),
      [](corex::core::Point ptA, corex::core::Point ptB) -> bool {
        return ptA.y > ptB.y;
      }
    );

    std::default_random_engine randGenerator;
    std::uniform_real_distribution<float> xPosDistribution{ minX, maxX };
    std::uniform_real_distribution<float> yPosDistribution{ minY, maxY };
    std::uniform_real_distribution<float> rotationDistribution{ 0.f, 360.f };

    Solution solution{ inputBuildings.size() };
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      solution.setBuildingXPos(i, xPosDistribution(randGenerator));
      solution.setBuildingYPos(i, yPosDistribution(randGenerator));
      solution.setBuildingRotation(i, rotationDistribution(randGenerator));
    }

    return solution;
  }

  double GA::getSolutionFitness(Solution& solution)
  {
    double fitness = 0.0;

    // For now, consider the objective function as the minimization of the
    // distance of the buildings from each other.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      for (int32_t j = 1; i < solution.getNumBuildings(); i++) {
        fitness += static_cast<double>(corex::core::distance2D(
          corex::core::Point{
            solution.getBuildingXPos(i),
            solution.getBuildingYPos(i)
          },
          corex::core::Point{
            solution.getBuildingXPos(j),
            solution.getBuildingYPos(j)
          }
        ));
      }
    }

    return fitness;
  }
}

#endif
