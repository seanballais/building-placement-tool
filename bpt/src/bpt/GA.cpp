#include <algorithm>
#include <cassert>
#include <cmath>
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

    this->recentRunAvgFitnesses.clear();
    this->recentRunBestFitnesses.clear();
    this->recentRunWorstFitnesses.clear();

    for (int32_t i = 0; i < populationSize; i++) {
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea);
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
        auto children = this->crossoverSolutions(parentA, parentB);
        newPopulation[numOffsprings] = children[0];
        
        float mutationProbability = corex::core::generateRandomReal(
          mutationChanceDistribution);
        if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
          this->mutateSolution(newPopulation[numOffsprings], boundingArea);
        }

        this->applySwapping(newPopulation[numOffsprings], inputBuildings);

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
              this->mutateSolution(newPopulation[weakestSolutionIndex],
                                   boundingArea);
            }

            this->applySwapping(newPopulation[weakestSolutionIndex],
                                inputBuildings);
          }
        } else {
          newPopulation[numOffsprings] = children[1];
          
          float mutationProbability = corex::core::generateRandomReal(
            mutationChanceDistribution);
          if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
            this->mutateSolution(newPopulation[numOffsprings], boundingArea);
          }

          this->applySwapping(newPopulation[numOffsprings], inputBuildings);

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
          fitness += corex::core::polygonArea(overlapPoly) * 1000;
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

    std::cout << "x: [" << minX << ", " << maxX << "]" << std::endl;
    std::cout << "y: [" << minY << ", " << maxY << "]" << std::endl;

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

  eastl::array<Solution, 2> GA::crossoverSolutions(const Solution& solutionA,
                                                   const Solution& solutionB)
  {
    // We're doing uniform crossover.
    std::uniform_int_distribution<int32_t> parentDistribution{ 0, 1 };
    int32_t numGenes = solutionA.getNumBuildings();

    // Prevent unnecessary copying of the parents.
    eastl::array<const Solution* const, 2> parents{ &solutionA, &solutionB };

    eastl::array<Solution, 2> children{ solutionA, solutionB };
    for (int32_t childIdx = 0; childIdx < children.size(); childIdx++) {
      for (int32_t geneIdx = 0; geneIdx < numGenes; geneIdx++) {
        int32_t parentIdx = corex::core::generateRandomInt(parentDistribution);
        const Solution* const parent = parents[parentIdx];

        children[childIdx].setBuildingXPos(geneIdx,
                                           parent->getBuildingXPos(geneIdx));
        children[childIdx].setBuildingYPos(geneIdx,
                                           parent->getBuildingYPos(geneIdx));
        children[childIdx].setBuildingRotation(
          geneIdx,
          parent->getBuildingRotation(geneIdx));
      }
    }

    return children;
  }

  void GA::mutateSolution(Solution& solution,
                          const corex::core::NPolygon& boundingArea)
  {
    std::uniform_int_distribution<int32_t> geneDistribution{
      0, solution.getNumBuildings() - 1
    };

    int32_t targetGeneIndex = corex::core::generateRandomInt(geneDistribution);

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

    float newXPos = corex::core::generateRandomReal(xPosDistribution);
    float newYPos = corex::core::generateRandomReal(yPosDistribution);
    float newRotation = corex::core::generateRandomReal(rotationDistribution);

    solution.setBuildingXPos(targetGeneIndex, newXPos);
    solution.setBuildingYPos(targetGeneIndex, newYPos);
    solution.setBuildingRotation(targetGeneIndex, newRotation);
  }

  void GA::applySwapping(Solution& solution,
                         const eastl::vector<InputBuilding>& inputBuildings)
  {
    eastl::vector<Solution> generatedSolutions;

    // Save original solution.
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        float building0XPos = solution.getBuildingXPos(i);
        float building0YPos = solution.getBuildingYPos(i);
        float building0Rot = solution.getBuildingRotation(i);

        float building1XPos = solution.getBuildingXPos(j);
        float building1YPos = solution.getBuildingYPos(j);
        float building1Rot = solution.getBuildingRotation(j);

        Solution altSolution0 = solution;
        Solution altSolution1 = solution;
        Solution altSolution2 = solution;

        // Swap the buildings' positions, and save as a possible solution.
        altSolution0.setBuildingXPos(i, building1XPos);
        altSolution0.setBuildingYPos(i, building1YPos);
        altSolution0.setBuildingXPos(j, building0XPos);
        altSolution0.setBuildingYPos(j, building0YPos);

        // Swap the buildings' rotations, and save as a possible solution.
        altSolution1.setBuildingRotation(i, building1Rot);
        altSolution1.setBuildingRotation(j, building0Rot);

        // Swap the buildings' positions and rotations, and save as a possible
        // solution.
        altSolution2.setBuildingXPos(i, building1XPos);
        altSolution2.setBuildingYPos(i, building1YPos);
        altSolution2.setBuildingXPos(j, building0XPos);
        altSolution2.setBuildingYPos(j, building0YPos);
        altSolution2.setBuildingRotation(i, building1Rot);
        altSolution2.setBuildingRotation(j, building0Rot);

        generatedSolutions.push_back(altSolution0);
        generatedSolutions.push_back(altSolution1);
        generatedSolutions.push_back(altSolution2);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [this, inputBuildings](Solution solutionA, Solution solutionB) {
        return corex::core::floatLessThan(
          this->getSolutionFitness(solutionA, inputBuildings),
          this->getSolutionFitness(solutionB, inputBuildings));
      }
    );
  }
}
