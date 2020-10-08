#include <algorithm>
#include <cassert>
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

#include <corex/core/math_functions.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Rectangle.hpp>

#include <bpt/GA.hpp>

namespace bpt
{
  GA::GA()
    : recentRunAvgFitnesses()
    , recentRunBestFitnesses()
    , recentRunWorstFitnesses() {}

  Solution GA::generateSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const float mutationRate,
    const int32_t populationSize,
    const int32_t numGenerations,
    const int32_t tournamentSize)
  {
    eastl::vector<Solution> population(populationSize);
    eastl::vector<double> populationFitnessValues(populationSize, 0.0);

    this->recentRunAvgFitnesses.clear();
    this->recentRunBestFitnesses.clear();
    this->recentRunWorstFitnesses.clear();

    for (int32_t i = 0; i < populationSize; i++) {
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea);
      populationFitnessValues[i] = this->getSolutionFitness(population[i],
                                                            inputBuildings);
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
      std::cout << "Generation: " << i << std::endl;

      int32_t numOffsprings = 0;
      eastl::vector<Solution> newPopulation(populationSize);
      while (numOffsprings < populationSize) {
        // Standard Tournament Selection.
        Solution parentA;
        Solution parentB;
        for (int32_t j = 0; j < tournamentSize; j++) {
          int32_t parentIndex = corex::core::generateRandomInt(
            chromosomeDistribution);
          if (j == 0
              || (corex::core::floatLessThan(
                   this->getSolutionFitness(population[parentIndex],
                                            inputBuildings),
                   this->getSolutionFitness(parentA, inputBuildings)))) {
            parentB = parentA;
            parentA = population[parentIndex];
          } else if ((parentB.getNumBuildings() == 0)
                     || (corex::core::floatLessThan(
                          this->getSolutionFitness(population[parentIndex],
                                                   inputBuildings),
                          this->getSolutionFitness(parentB, inputBuildings)))) {
            parentB = population[parentIndex];
          }
        }

        // Make sure we have individuals from the population, and not just
        // empty solutions.
        assert(parentA.getNumBuildings() != 0);
        assert(parentB.getNumBuildings() != 0);

        // Crossover
        auto children = parentA.crossover(parentB);
        newPopulation[numOffsprings] = children[0];
        
        float mutationProbability = corex::core::generateRandomReal(
          mutationChanceDistribution);
        if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
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
            [this, inputBuildings](Solution solutionA, Solution solutionB)
                                  -> bool {
              return corex::core::floatLessThan(
                this->getSolutionFitness(solutionA, inputBuildings),
                this->getSolutionFitness(solutionB, inputBuildings));
            }
          );

          if (corex::core::floatLessThan(
                this->getSolutionFitness(children[1], inputBuildings),
                this->getSolutionFitness(*weakestSolutionIter,
                                         inputBuildings))) {
            int32_t weakestSolutionIndex = std::distance(newPopulation.begin(),
                                                         weakestSolutionIter);
            newPopulation[weakestSolutionIndex] = children[1];

            float mutationProbability = corex::core::generateRandomReal(
              mutationChanceDistribution);
            if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
              newPopulation[weakestSolutionIndex].mutate();
            }
          }
        } else {
          newPopulation[numOffsprings] = children[1];
          
          float mutationProbability = corex::core::generateRandomReal(
            mutationChanceDistribution);
          if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
            newPopulation[numOffsprings].mutate();
          }

          numOffsprings++;
        }
      }

      // Replace the worst solution in the new population with the best solution
      // in the previous generation, if the previous best solution is better
      // than the current generation worst. We should only do this in the
      // second genetation.
      if (i > 0) {
        auto weakestSolutionIter = std::max_element(
          newPopulation.begin(),
          newPopulation.end(),
          [this, inputBuildings](Solution solutionA, Solution solutionB)
                                -> bool {
            return corex::core::floatLessThan(
              this->getSolutionFitness(solutionA, inputBuildings),
              this->getSolutionFitness(solutionB, inputBuildings));
            }
          );

        if (corex::core::floatLessThan(
              this->getSolutionFitness(bestSolution, inputBuildings),
              this->getSolutionFitness(*weakestSolutionIter,
                                       inputBuildings))) {
          int32_t weakestSolutionIndex = std::distance(newPopulation.begin(),
                                                       weakestSolutionIter);
          newPopulation[weakestSolutionIndex] = bestSolution;
        }
      }
      

      population = newPopulation;

      double fitnessAverage = 0.0;
      for (Solution& sol : population) {
        fitnessAverage += this->getSolutionFitness(sol, inputBuildings);
      }

      fitnessAverage = fitnessAverage / newPopulation.size();
      this->recentRunAvgFitnesses.push_back(static_cast<float>(fitnessAverage));

      bestSolution = *std::min_element(
        population.begin(),
        population.end(),
        [this, inputBuildings](Solution solutionA, Solution solutionB) {
          return corex::core::floatLessThan(
            this->getSolutionFitness(solutionA, inputBuildings),
            this->getSolutionFitness(solutionB, inputBuildings));
        }
      );

      this->recentRunBestFitnesses.push_back(static_cast<float>(
        this->getSolutionFitness(bestSolution, inputBuildings)));

      worstSolution = *std::max_element(
        population.begin(),
        population.end(),
        [this, inputBuildings](Solution solutionA, Solution solutionB) {
          return corex::core::floatLessThan(
            this->getSolutionFitness(solutionA, inputBuildings),
            this->getSolutionFitness(solutionB, inputBuildings));
        }
      );

      this->recentRunWorstFitnesses.push_back(static_cast<float>(
        this->getSolutionFitness(worstSolution, inputBuildings)));
    }

    return bestSolution;
  }

  double GA::getSolutionFitness(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings)
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

        auto rectA = corex::core::Rectangle{
          solution.getBuildingXPos(i),
          solution.getBuildingYPos(i),
          inputBuildings[i].width,
          inputBuildings[i].length,
          solution.getBuildingRotation(i)
        };
        auto rectB = corex::core::Rectangle{
          solution.getBuildingXPos(j),
          solution.getBuildingYPos(j),
          inputBuildings[j].width,
          inputBuildings[j].length,
          solution.getBuildingRotation(j)
        };
        if (corex::core::areTwoRectsIntersecting(rectA, rectB)) {
          auto overlapPoly = corex::core::clippedPolygonFromTwoRects(rectA,
                                                                     rectB);
          fitness += corex::core::polygonArea(overlapPoly);
        }
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
  GA::generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea)
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

    return solution;
  }
}
