#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <random>

#include <EASTL/array.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

#include <corex/core/utils.hpp>

#include <bpt/GA.hpp>

namespace bpt
{
  GA::GA()
    : recentRunAvgFitnesses()
    , recentRunBestFitnesses()
    , recentRunWorstFitnesses() {}

  Solution GA::generateSolution(eastl::vector<InputBuilding>& inputBuildings,
                                corex::core::NPolygon& boundingArea,
                                float mutationRate,
                                int32_t populationSize,
                                int32_t numGenerations)
  {
    eastl::vector<Solution> population(populationSize);
    eastl::vector<double> populationFitnessValues(populationSize, 0.0);

    this->recentRunAvgFitnesses.clear();
    this->recentRunBestFitnesses.clear();
    this->recentRunWorstFitnesses.clear();

    for (int32_t i = 0; i < populationSize; i++) {
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea);
      populationFitnessValues[i] = this->getSolutionFitness(population[i]);
    }

    std::uniform_int_distribution<int32_t> chromosomeDistribution{
      0, populationSize - 1
    };
    std::uniform_real_distribution<float> mutationChanceDistribution{
      0.f, 1.f
    };

    Solution bestSolution;
    Solution worstSolution;
    for (int32_t i = 0; i < numGenerations; i++) {
      // Selection
      // Preprocessing for roulette wheel selection. Compute probability of each
      // solution.
      double totalFitness = 0.0;
      for (Solution& solution : population) {
        totalFitness += this->getSolutionFitness(solution);
      }

      int32_t numOffsprings = 0;
      eastl::vector<Solution> newPopulation(populationSize);
      while (numOffsprings < populationSize) {
        int32_t parentAIndex = corex::core::generateRandomInt(
          chromosomeDistribution);
        int32_t parentBIndex = 0;
        do {
          parentBIndex = corex::core::generateRandomInt(chromosomeDistribution);
        } while (parentBIndex == parentAIndex);

        // Roulette Wheel Selection.


        // Crossover
        auto children = population[parentAIndex].crossover(
          population[parentBIndex]);

        newPopulation[numOffsprings] = children[0];
        
        float mutationProbability = corex::core::generateRandomReal(
          mutationChanceDistribution);
        if (mutationProbability < mutationRate) {
          newPopulation[numOffsprings].mutate();
        }

        numOffsprings++;

        // In cases where the population size is not an even number, a child
        // will have to be dropped. As such, we'll only add the second
        // generated child if it has a fitness better than the worst solution
        // in the new generation.
        if (numOffsprings == populationSize) {
          auto weakestSolutionIter = std::max_element(
            newPopulation.begin(),
            newPopulation.end(),
            [this](Solution solutionA, Solution solutionB) -> bool {
              return this->getSolutionFitness(solutionA)
                < this->getSolutionFitness(solutionB);
            }
          );

          if (this->getSolutionFitness(children[1])
              < this->getSolutionFitness(*weakestSolutionIter)) {
            int32_t weakestSolutionIndex = std::distance(newPopulation.begin(),
                                                         weakestSolutionIter);
            newPopulation[weakestSolutionIndex] = children[1];

            float mutationProbability = corex::core::generateRandomReal(
              mutationChanceDistribution);
            if (mutationProbability < mutationRate) {
              newPopulation[weakestSolutionIndex].mutate();
            }
          }
        } else {
          newPopulation[numOffsprings] = children[1];
          
          float mutationProbability = corex::core::generateRandomReal(
            mutationChanceDistribution);
          if (mutationProbability < mutationRate) {
            newPopulation[numOffsprings].mutate();
          }

          numOffsprings++;
        }
      }

      population = newPopulation;

      double fitnessAverage = 0.0;
      for (Solution& sol : population) {
        fitnessAverage += this->getSolutionFitness(sol);
      }

      fitnessAverage = fitnessAverage / newPopulation.size();
      this->recentRunAvgFitnesses.push_back(static_cast<float>(fitnessAverage));

      bestSolution = *std::min_element(
        population.begin(),
        population.end(),
        [this](Solution solutionA, Solution solutionB) {
          return this->getSolutionFitness(solutionA)
                 < this->getSolutionFitness(solutionB);
        }
      );

      this->recentRunBestFitnesses.push_back(static_cast<float>(
        this->getSolutionFitness(bestSolution)));

      worstSolution = *std::max_element(
        population.begin(),
        population.end(),
        [this](Solution solutionA, Solution solutionB) {
          return this->getSolutionFitness(solutionA)
                 < this->getSolutionFitness(solutionB);
        }
      );

      this->recentRunWorstFitnesses.push_back(static_cast<float>(
        this->getSolutionFitness(worstSolution)));
    }

    return bestSolution;
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

  eastl::vector<float> GA::getRecentRunAverageFitnesses()
  {
    return this->recentRunAvgFitnesses;
  }

  eastl::vector<float> GA::getRecentRunBestFitnesses()
  {
    return this->recentRunBestFitnesses;
  }

  eastl::vector<float> GA::getRecentRunWorstFitnesses()
  {
    return this->recentRunWorstFitnesses;
  }

  Solution
  GA::generateRandomSolution(eastl::vector<InputBuilding>& inputBuildings,
                             corex::core::NPolygon& boundingArea)
  {
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

    std::cout << "minX: " << minX << std::endl;
    std::cout << "maxX: " << maxX << std::endl;
    std::cout << "minY: " << minY << std::endl;
    std::cout << "maxY: " << maxY << std::endl;

    std::uniform_real_distribution<float> xPosDistribution{ minX, maxX };
    std::uniform_real_distribution<float> yPosDistribution{ minY, maxY };
    std::uniform_real_distribution<float> rotationDistribution{ 0.f, 360.f };

    Solution solution{ static_cast<int32_t>(inputBuildings.size()) };
    for (int32_t i = 0; i < inputBuildings.size(); i++) {
      solution.setBuildingXPos(
        i,
        corex::core::generateRandomReal(xPosDistribution));
      solution.setBuildingYPos(
        i,
        corex::core::generateRandomReal(yPosDistribution));
      solution.setBuildingRotation(
        i,
        corex::core::generateRandomReal(rotationDistribution));
    }

    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      std::cout << "Building #" << i << std::endl;
      std::cout << "-- x: " << solution.getBuildingXPos(i) << std::endl;
      std::cout << "-- y: " << solution.getBuildingYPos(i) << std::endl;
      std::cout << "-- Rotation: " << solution.getBuildingRotation(i) << std::endl;
    }

    std::cout << "Random Solution Fitness: " << this->getSolutionFitness(solution) << std::endl;

    return solution;
  }
}
