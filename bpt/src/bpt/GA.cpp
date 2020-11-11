#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <random>

#include <EASTL/array.h>
#include <EASTL/functional.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>

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

  Solution GA::generateSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    eastl::vector<corex::core::NPolygon>& floodProneAreas,
    eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float mutationRate,
    const int32_t populationSize,
    const int32_t numGenerations,
    const int32_t tournamentSize,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const bool isLocalSearchEnabled)
  {
    assert(flowRates.size() == inputBuildings.size());

    eastl::vector<Solution> population(populationSize);

    this->recentRunAvgFitnesses.clear();
    this->recentRunBestFitnesses.clear();
    this->recentRunWorstFitnesses.clear();

    std::cout << "|| Generating Initial Population..." << std::endl;
    for (int32_t i = 0; i < populationSize; i++) {
      std::cout << "Generating solution #" << i << "..." << std::endl;
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea);
      population[i].setFitness(
        this->getSolutionFitness(
          population[i],
          inputBuildings,
          flowRates,
          floodProneAreas,
          landslideProneAreas,
          floodProneAreaPenalty,
          landslideProneAreaPenalty));
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
          if (j == 0 // Boolean short-circuit. Hehe.
              || corex::core::floatLessThan(
                  population[parentIndex].getFitness(),
                  parentA.getFitness())) {
            parentB = parentA;
            parentA = population[parentIndex];
          } else if (parentB.getNumBuildings() == 0 // Boolean short again.
                     || corex::core::floatLessThan(
                          population[parentIndex].getFitness(),
                          parentB.getFitness())) {
            parentB = population[parentIndex];
          }
        }

        // Make sure we have individuals from the population, and not just
        // empty solutions.
        assert(parentA.getNumBuildings() != 0);
        assert(parentB.getNumBuildings() != 0);

        // Crossover
        auto children = this->crossoverSolutions(parentA,
                                                 parentB,
                                                 boundingArea,
                                                 inputBuildings);
        newPopulation[numOffsprings] = children[0];
        newPopulation[numOffsprings].setFitness(this->getSolutionFitness(
          newPopulation[numOffsprings],
          inputBuildings,
          flowRates,
          floodProneAreas,
          landslideProneAreas,
          floodProneAreaPenalty,
          landslideProneAreaPenalty));
        
        float mutationProbability = corex::core::generateRandomReal(
          mutationChanceDistribution);
        if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
          this->mutateSolution(newPopulation[numOffsprings],
                               boundingArea,
                               inputBuildings);
          newPopulation[numOffsprings].setFitness(this->getSolutionFitness(
            newPopulation[numOffsprings],
            inputBuildings,
            flowRates,
            floodProneAreas,
            landslideProneAreas,
            floodProneAreaPenalty,
            landslideProneAreaPenalty));
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
            [](Solution solutionA, Solution solutionB) -> bool {
              return corex::core::floatLessThan(solutionA.getFitness(),
                                                solutionB.getFitness());
            }
          );

          if (corex::core::floatLessThan(children[1].getFitness(),
                                         weakestSolutionIter->getFitness())) {
            int32_t weakestSolutionIndex = std::distance(newPopulation.begin(),
                                                         weakestSolutionIter);
            newPopulation[weakestSolutionIndex] = children[1];
            newPopulation[weakestSolutionIndex].setFitness(
              this->getSolutionFitness(
                newPopulation[weakestSolutionIndex],
                inputBuildings,
                flowRates,
                floodProneAreas,
                landslideProneAreas,
                floodProneAreaPenalty,
                landslideProneAreaPenalty));

            float mutationProbability = corex::core::generateRandomReal(
              mutationChanceDistribution);
            if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
              this->mutateSolution(newPopulation[weakestSolutionIndex],
                                   boundingArea,
                                   inputBuildings);
              newPopulation[weakestSolutionIndex].setFitness(
                this->getSolutionFitness(
                  newPopulation[weakestSolutionIndex],
                  inputBuildings,
                  flowRates,
                  floodProneAreas,
                  landslideProneAreas,
                  floodProneAreaPenalty,
                  landslideProneAreaPenalty));
            }
          }
        } else {
          newPopulation[numOffsprings] = children[1];
          newPopulation[numOffsprings].setFitness(this->getSolutionFitness(
            newPopulation[numOffsprings],
            inputBuildings,
            flowRates,
            floodProneAreas,
            landslideProneAreas,
            floodProneAreaPenalty,
            landslideProneAreaPenalty));
          
          float mutationProbability = corex::core::generateRandomReal(
            mutationChanceDistribution);
          if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
            this->mutateSolution(newPopulation[numOffsprings],
                                 boundingArea,
                                 inputBuildings);
            newPopulation[numOffsprings].setFitness(this->getSolutionFitness(
              newPopulation[numOffsprings],
              inputBuildings,
              flowRates,
              floodProneAreas,
              landslideProneAreas,
              floodProneAreaPenalty,
              landslideProneAreaPenalty));
          }

          numOffsprings++;
        }
      }

      // Replace the worst solution in the new population with the best solution
      // in the previous generation, if the previous best solution is better
      // than the current generation worst. We should only do this in the
      // second generation.
      if (i > 0) {
        auto weakestSolutionIter = std::max_element(
          newPopulation.begin(),
          newPopulation.end(),
          [](Solution solutionA, Solution solutionB) -> bool {
            return corex::core::floatLessThan(solutionA.getFitness(),
                                              solutionB.getFitness());
          }
        );

        if (corex::core::floatLessThan(bestSolution.getFitness(),
                                       weakestSolutionIter->getFitness())) {
          int32_t weakestSolutionIndex = std::distance(newPopulation.begin(),
                                                       weakestSolutionIter);
          newPopulation[weakestSolutionIndex] = bestSolution;
        }
      }

      population = newPopulation;

      bestSolution = *std::min_element(
        population.begin(),
        population.end(),
        [](Solution solutionA, Solution solutionB) {
          return corex::core::floatLessThan(solutionA.getFitness(),
                                            solutionB.getFitness());
        }
      );

      if (isLocalSearchEnabled) {
        this->applyLocalSearch1(bestSolution,
                                boundingArea,
                                inputBuildings,
                                flowRates);
        bestSolution.setFitness(this->getSolutionFitness(
          bestSolution,
          inputBuildings,
          flowRates,
          floodProneAreas,
          landslideProneAreas,
          floodProneAreaPenalty,
          landslideProneAreaPenalty));
      }

      double fitnessAverage = 0.0;
      for (Solution& sol : population) {
        fitnessAverage += sol.getFitness();
      }

      fitnessAverage = fitnessAverage / newPopulation.size();
      this->recentRunAvgFitnesses.push_back(static_cast<float>(fitnessAverage));

      this->recentRunBestFitnesses.push_back(static_cast<float>(
        bestSolution.getFitness()));

      worstSolution = *std::max_element(
        population.begin(),
        population.end(),
        [](Solution solutionA, Solution solutionB) {
          return corex::core::floatLessThan(solutionA.getFitness(),
                                            solutionB.getFitness());
        }
      );

      this->recentRunWorstFitnesses.push_back(static_cast<float>(
        worstSolution.getFitness()));
    }

    return bestSolution;
  }

  double GA::getSolutionFitness(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty)
  {
    double fitness = 0.0;

    // Compute fitness for the inter-building distance part.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      assert(flowRates[i].size() == solution.getNumBuildings());
      for (int32_t j = 1; j < solution.getNumBuildings(); j++) {
        if (i == j) {
          continue;
        }

        fitness += static_cast<double>(
          corex::core::distance2D(corex::core::Point{
                                    solution.getBuildingXPos(i),
                                    solution.getBuildingYPos(i)
                                  },
                                  corex::core::Point{
                                    solution.getBuildingXPos(j),
                                    solution.getBuildingYPos(j)
                                  })
          * flowRates[i][j]
        );
      }
    }

    // Compute penalty for placing buildings in hazard areas.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      corex::core::Rectangle building {
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };
      // Compute penalty for placing a building in a flood-prone area.
      for (const corex::core::NPolygon& area : floodProneAreas) {
        if (corex::core::isRectIntersectingNPolygon(building, area)) {
          fitness += floodProneAreaPenalty;
        }
      }

      // Compute penalty for placing a building in a landslide-prone area.
      for (const corex::core::NPolygon& area : landslideProneAreas) {
        if (corex::core::isRectIntersectingNPolygon(building, area)) {
          fitness += landslideProneAreaPenalty;
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
    do {
      for (int32_t i = 0; i < inputBuildings.size(); i++) {
        corex::core::Point buildingPos { 0.f, 0.f };
        float buildingRotation = 0.f;
        corex::core::Rectangle buildingRect {
          buildingPos.x,
          buildingPos.y,
          inputBuildings[i].width,
          inputBuildings[i].length,
          buildingRotation
        };
        do {
          buildingPos.x = corex::core::generateRandomReal(xPosDistribution);
          buildingPos.y = corex::core::generateRandomReal(yPosDistribution);
          buildingRotation = corex::core::generateRandomReal(
            rotationDistribution);
          buildingRect.x = buildingPos.x;
          buildingRect.y = buildingPos.y;
          buildingRect.angle = buildingRotation;
        } while (!isRectWithinNPolygon(buildingRect, boundingArea));

        solution.setBuildingXPos(i, buildingPos.x);
        solution.setBuildingYPos(i, buildingPos.y);
        solution.setBuildingRotation(i, buildingRotation);
      }
    } while (!this->isSolutionFeasible(solution, boundingArea, inputBuildings));

    return solution;
  }

  eastl::array<Solution, 2>
  GA::crossoverSolutions(const Solution& solutionA,
                         const Solution& solutionB,
                         const corex::core::NPolygon& boundingArea,
                         const eastl::vector<InputBuilding>& inputBuildings)
  {
    // We're doing uniform crossover.
    std::uniform_int_distribution<int32_t> parentDistribution{ 0, 1 };
    int32_t numGenes = solutionA.getNumBuildings();

    // Prevent unnecessary copying of the parents.
    eastl::array<const Solution* const, 2> parents{ &solutionA, &solutionB };

    eastl::array<Solution, 2> children{ solutionA, solutionB };
    for (int32_t childIdx = 0; childIdx < children.size(); childIdx++) {
      do {      
        for (int32_t geneIdx = 0; geneIdx < numGenes; geneIdx++) {
          int32_t parentIdx = corex::core::generateRandomInt(
            parentDistribution);
          const Solution* const parent = parents[parentIdx];

          children[childIdx].setBuildingXPos(geneIdx,
                                             parent->getBuildingXPos(geneIdx));
          children[childIdx].setBuildingYPos(geneIdx,
                                             parent->getBuildingYPos(geneIdx));
          children[childIdx].setBuildingRotation(
            geneIdx,
            parent->getBuildingRotation(geneIdx));
        }
      } while (!this->isSolutionFeasible(children[childIdx],
                                         boundingArea,
                                         inputBuildings));
    }

    return children;
  }

  void GA::mutateSolution(Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings)
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

    Solution tempSolution = solution;
    do {
      float newXPos = corex::core::generateRandomReal(xPosDistribution);
      float newYPos = corex::core::generateRandomReal(yPosDistribution);
      float newRotation = corex::core::generateRandomReal(rotationDistribution);

      tempSolution.setBuildingXPos(targetGeneIndex, newXPos);
      tempSolution.setBuildingYPos(targetGeneIndex, newYPos);
      tempSolution.setBuildingRotation(targetGeneIndex, newRotation);
    } while (!this->isSolutionFeasible(tempSolution,
                                       boundingArea,
                                       inputBuildings));

    solution = tempSolution;
  }

  void GA::applyLocalSearch1(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates)
  {
    constexpr int32_t numMovements = 8;
    constexpr float maxShiftAmount = 15.f;
    std::uniform_real_distribution<float> shiftDistrib{ 0, maxShiftAmount };
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t)>,
                 numMovements> searchFunctions = {
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmount);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 - shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 + shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 - shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 - shiftAmountB);
        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        float shiftAmountA = corex::core::generateRandomReal(shiftDistrib);
        float shiftAmountB = corex::core::generateRandomReal(shiftDistrib);
        solution.setBuildingXPos(buildingIndex,
                                 solution.getBuildingXPos(buildingIndex)
                                 + shiftAmountA);

        // NOTE: The origin is on the top left corner.
        solution.setBuildingYPos(buildingIndex,
                                 solution.getBuildingYPos(buildingIndex)
                                 + shiftAmountB);
        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    constexpr float maxRotShiftAmount = 90.f;
    std::uniform_real_distribution<float> rotShiftDistrib{
      -maxRotShiftAmount,
      maxRotShiftAmount
    };

    for (int32_t movementID = 0; movementID < numMovements; movementID++) {
      int32_t buildingIndex = movementID % inputBuildings.size();

      auto altSolution0 = searchFunctions[movementID](solution, buildingIndex);
      if (this->isSolutionFeasible(altSolution0,
                                   boundingArea,
                                   inputBuildings)) {
        generatedSolutions.push_back(altSolution0);
      }

      auto altSolution1 = altSolution0;
      solution.setBuildingRotation(
        buildingIndex,
        corex::core::generateRandomReal(rotShiftDistrib));
      if (this->isSolutionFeasible(altSolution1,
                                   boundingArea,
                                   inputBuildings)) {
        generatedSolutions.push_back(altSolution1);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](Solution solutionA, Solution solutionB) {
        return corex::core::floatLessThan(solutionA.getFitness(),
                                          solutionB.getFitness());
      }
    );
  }

  bool
  GA::isSolutionFeasible(const Solution& solution,
                         const corex::core::NPolygon& boundingArea,
                         const eastl::vector<InputBuilding>& inputBuildings)
  {
    return this->doesSolutionHaveNoBuildingsOverlapping(solution,
                                                        inputBuildings)
           && this->areSolutionBuildingsWithinBounds(solution,
                                                     boundingArea,
                                                     inputBuildings);
  }

  bool GA::doesSolutionHaveNoBuildingsOverlapping(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      corex::core::Rectangle building0 = corex::core::Rectangle{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };

      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        corex::core::Rectangle building1 = corex::core::Rectangle{
          solution.getBuildingXPos(j),
          solution.getBuildingYPos(j),
          inputBuildings[j].width,
          inputBuildings[j].length,
          solution.getBuildingRotation(j)
        };
        if (corex::core::areTwoRectsIntersecting(building0, building1)) {
          return false;
        }
      }
    }

    return true;
  }

  bool GA::areSolutionBuildingsWithinBounds(
    const Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      corex::core::Rectangle buildingRect = corex::core::Rectangle{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };

      if (!corex::core::isRectWithinNPolygon(buildingRect, boundingArea)) {
        return false;
      }
    }

    return true;
  }
}
