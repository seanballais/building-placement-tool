#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <random>

#include <EASTL/array.h>
#include <EASTL/functional.h>
#include <EASTL/memory.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/Line.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/utils.hpp>

#include <iprof/iprof.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/GA.hpp>
#include <bpt/GD.hpp>
#include <bpt/generator.hpp>
#include <bpt/operators.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  GA::GA()
    : currRunGenerationNumber(-1)
    , recentRunAvgFitnesses()
    , recentRunBestFitnesses()
    , recentRunWorstFitnesses()
    , runTimer()
    , recentRunElapsedTime(0.0)
    , hillClimbing() {}

  eastl::vector<eastl::vector<Solution>> GA::generateSolutions(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    eastl::vector<corex::core::NPolygon>& floodProneAreas,
    eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float mutationRate,
    const int32_t populationSize,
    const int32_t numGenerations,
    const int32_t tournamentSize,
    const int32_t numPrevGenOffsprings,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const bool isLocalSearchEnabled,
    const CrossoverType crossoverType,
    const SelectionType selectionType,
    const int32_t numIters,
    const bool& keepInfeasibleSolutions)
  {
    assert(flowRates.size() == inputBuildings.size());

    this->runTimer.start();

    eastl::vector<eastl::vector<Solution>> solutions;

    this->recentRunAvgFitnesses.clear();
    this->recentRunBestFitnesses.clear();
    this->recentRunWorstFitnesses.clear();

    eastl::vector<Solution> population(populationSize);
    population = this->generateInitialPopulation(populationSize,
                                                 inputBuildings,
                                                 boundingArea,
                                                 flowRates,
                                                 floodProneAreas,
                                                 landslideProneAreas,
                                                 floodProneAreaPenalty,
                                                 landslideProneAreaPenalty,
                                                 buildingDistanceWeight);

    // Add the initial population.
    solutions.push_back(population);

    Solution bestSolution = *std::min_element(
      population.begin(),
      population.end(),
      [](Solution solutionA, Solution solutionB) {
        return corex::core::floatLessThan(solutionA.getFitness(),
                                          solutionB.getFitness());
      }
    );
    Solution worstSolution = *std::max_element(
      population.begin(),
      population.end(),
      [](Solution solutionA, Solution solutionB) {
        return corex::core::floatLessThan(solutionA.getFitness(),
                                          solutionB.getFitness());
      }
    );

    // Add statistics about the initial population.
    double fitnessAverage = 0.0;
    for (Solution& sol : population) {
      fitnessAverage += sol.getFitness();
    }

    fitnessAverage = fitnessAverage / population.size();
    this->recentRunAvgFitnesses.push_back(fitnessAverage);
    this->recentRunBestFitnesses.push_back(bestSolution.getFitness());
    this->recentRunWorstFitnesses.push_back(worstSolution.getFitness());

    const int32_t numOffspringsToMake = populationSize - numPrevGenOffsprings;
    for (int32_t i = 0; i < numGenerations; i++) {
      this->currRunGenerationNumber++;

      std::cout << "Generation #" << this->currRunGenerationNumber << "\n";

      if (i < 100) {
        // We apply the swapping method to the first 100 generations.
        std::cout << "Applying the swapping method.\n";

        for (Solution& offspring : population) {
          applySwappingMethod(offspring, boundingArea, inputBuildings);
        }
      }

      int32_t numOffsprings = 0;
      eastl::vector<Solution> newOffsprings(numOffspringsToMake);
      while (numOffsprings < numOffspringsToMake) {
        auto parents = this->selectParents(population,
                                           tournamentSize,
                                           selectionType);
        Solution parentA = parents[0];
        Solution parentB = parents[1];

        // Make sure we have individuals from the population, and not just
        // empty solutions.
        assert(parentA.getNumBuildings() != 0);
        assert(parentB.getNumBuildings() != 0);

        // Breeding time.
        this->makeTwoParentsBreed(
          parentA,
          parentB,
          crossoverType,
          newOffsprings,
          numOffsprings,
          numOffspringsToMake,
          mutationRate,
          boundingArea,
          inputBuildings,
          flowRates,
          floodProneAreas,
          landslideProneAreas,
          floodProneAreaPenalty,
          landslideProneAreaPenalty,
          buildingDistanceWeight,
          keepInfeasibleSolutions);
      }

      std::sort(
        population.begin(),
        population.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return corex::core::floatLessThan(solutionA.getFitness(),
                                            solutionB.getFitness());
        }
      );

      // Keep only a set number of offsprings from the previous generation.
      // The best few offsprings from the previous generation will be placed
      // at the front of the population vector at the end of the loop below.
      for (int32_t i = numPrevGenOffsprings; i < population.size(); i++) {
        population[i] = newOffsprings[i - numPrevGenOffsprings];
      }

      Solution& currGenBest = *std::min_element(
        population.begin() + numPrevGenOffsprings,
        population.end(),
        [](const Solution& a, const Solution& b) {
          return a.getFitness() < b.getFitness();
        }
      );

      this->applyLocalSearch1(currGenBest,
                              boundingArea,
                              inputBuildings,
                              flowRates,
                              buildingDistanceWeight);

      if (i >= numGenerations - 50) {
        this->applyLocalSearch2(currGenBest,
                                boundingArea,
                                inputBuildings,
                                flowRates,
                                buildingDistanceWeight);
      }

      std::sort(
        population.begin(),
        population.end(),
        [](Solution& solutionA, Solution& solutionB) {
          return corex::core::floatLessThan(solutionA.getFitness(),
                                            solutionB.getFitness());
        }
      );

      bestSolution = population[0];
      bestSolution.setFitness(computeSolutionFitness(bestSolution,
                                                     inputBuildings,
                                                     boundingArea,
                                                     flowRates,
                                                     buildingDistanceWeight));

      solutions.push_back(population);

      double fitnessAverage = 0.0;
      for (Solution& sol : population) {
        fitnessAverage += sol.getFitness();
      }

      fitnessAverage = fitnessAverage / population.size();
      this->recentRunAvgFitnesses.push_back(static_cast<float>(fitnessAverage));

      this->recentRunBestFitnesses.push_back(static_cast<float>(
        bestSolution.getFitness()));

      worstSolution = population.back();

      this->recentRunWorstFitnesses.push_back(static_cast<float>(
        worstSolution.getFitness()));

      InternalProfiler::aggregateEntries();
      std::cout << "The latest internal profiler stats:\n"
                << InternalProfiler::stats << std::endl;
    }

    if (isLocalSearchEnabled) {
      auto lsGeneratedSolutions = this->hillClimbing.generateSolution(
        bestSolution,
        inputBuildings,
        boundingArea,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight,
        numIters,
        &(this->currRunGenerationNumber));

      for (const eastl::vector<Solution>& solution : lsGeneratedSolutions) {
        this->recentRunBestFitnesses.push_back(solution[0].getFitness());
      }

      solutions.insert(solutions.end(),
                       lsGeneratedSolutions.begin(),
                       lsGeneratedSolutions.end());
    }

    this->currRunGenerationNumber = -1;

    this->recentRunElapsedTime = this->runTimer.getElapsedTime();
    this->runTimer.stop();

    return solutions;
  }

  int32_t GA::getCurrentRunIterationNumber()
  {
    return this->currRunGenerationNumber;
  }

  eastl::vector<double> GA::getRecentRunAverageFitnesses()
  {
    return this->recentRunAvgFitnesses;
  }

  eastl::vector<double> GA::getRecentRunBestFitnesses()
  {
    return this->recentRunBestFitnesses;
  }

  eastl::vector<double> GA::getRecentRunWorstFitnesses()
  {
    return this->recentRunWorstFitnesses;
  }

  double GA::getRecentRunElapsedTime()
  {
    // We're gonna return in terms of seconds.
    return this->recentRunElapsedTime;
  }

  eastl::vector<Solution>
  GA::generateInitialPopulation(
    const int32_t populationSize,
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
    eastl::vector<Solution> population(populationSize);
    std::cout << "|| Generating Initial Population..." << std::endl;
    for (int32_t i = 0; i < populationSize; i++) {
      std::cout << "Generating solution #" << i << "..." << std::endl;
      population[i] = generateRandomSolution(inputBuildings, boundingArea);
      population[i].setFitness(computeSolutionFitness(population[i],
                                                      inputBuildings,
                                                      boundingArea,
                                                      flowRates,
                                                      buildingDistanceWeight));
    }

    return population;
  }

  eastl::array<Solution, 2> GA::selectParents(
    const eastl::vector<Solution>& population,
    const int32_t& tournamentSize,
    const SelectionType& selectionType)
  {
    eastl::array<Solution, 2> parents;
    switch (selectionType) {
      case SelectionType::RS:
        parents = this->runRankedSelection(population);
        break;
      case SelectionType::RWS:
        parents = this->runRouletteWheelSelection(population);
        break;
      case SelectionType::TS:
        parents = this->runTournamentSelection(population, tournamentSize);
        break;
      default:
        break;
    }

    return parents;
  }

  eastl::array<Solution, 2> GA::runRouletteWheelSelection(
    const eastl::vector<Solution>& population)
  {
    // Let's try roulette wheel selection. Code based from:
    //   https://stackoverflow.com/a/26316267/1116098
    eastl::vector<double> popFitnesses;
    for (int32_t i = 0; i < population.size(); i++) {
      popFitnesses.push_back(population[i].getFitness());
    }

    double fitnessSum = std::accumulate(popFitnesses.begin(),
                                        popFitnesses.end(),
                                        0);
    double maxFitness = *std::max_element(
      popFitnesses.begin(),
      popFitnesses.end(),
      [](double a, double b) {
        return corex::core::floatLessEqual(a, b);
      });
    double minFitness = *std::min_element(
      popFitnesses.begin(),
      popFitnesses.end(),
      [](double a, double b) {
        return corex::core::floatLessEqual(a, b);
      });
    double upperBound = maxFitness + minFitness;

    std::uniform_real_distribution<double> fitnessDistrib {0, fitnessSum };

    eastl::array<Solution, 2> parents;
    for (int32_t i = 0; i < parents.size(); i++) {
      double p = corex::core::generateRandomReal(fitnessDistrib);
      parents[i] = population[0]; // Default selection.
      for (int32_t j = 0; j < popFitnesses.size(); j++) {
        p -= upperBound - popFitnesses[i];

        if (corex::core::floatLessEqual(p, 0.f)) {
          parents[i] = population[j];
          break;
        }
      }
    }

    return parents;
  }

  eastl::array<Solution, 2> GA::runRankedSelection(
    eastl::vector<Solution> population)
  {
    eastl::array<Solution, 2> parents;
    std::sort(
      population.begin(),
      population.end(),
      [](Solution& solutionA, Solution& solutionB) {
        return corex::core::floatLessThan(solutionA.getFitness(),
                                          solutionB.getFitness());
      }
    );

    eastl::vector<float> weights;
    for (int32_t i = 0; i < population.size(); i++) {
      weights.push_back(population.size() - i);
    }

    for (auto &parent : parents) {
      parent = cx::selectRandomItemWithWeights(population, weights);
    }

    return parents;
  }

  eastl::array<Solution, 2> GA::runTournamentSelection(
    const eastl::vector<Solution>& population,
    const int32_t& tournamentSize)
  {
    std::uniform_int_distribution<int32_t> chromosomeDistribution{
      0, static_cast<int32_t>(population.size() - 1)
    };

    eastl::array<Solution, 2> parents;
    for (int32_t j = 0; j < tournamentSize; j++) {
      int32_t parentIndex = corex::core::generateRandomInt(
        chromosomeDistribution);
      if (j == 0 // Boolean short-circuit. Hehe.
          || population[parentIndex].getFitness() < parents[0].getFitness()) {
        parents[1] = parents[0];
        parents[0] = population[parentIndex];
      } else if (parents[1].getNumBuildings() == 0 // Boolean short again.
                 || population[parentIndex].getFitness()
                    < parents[1].getFitness()) {
        parents[1] = population[parentIndex];
      }
    }

    return parents;
  }

  void GA::makeTwoParentsBreed(
    const Solution &parentA,
    const Solution &parentB,
    const CrossoverType& crossoverType,
    eastl::vector<Solution> &offsprings,
    int32_t &numOffsprings,
    const int32_t numOffspringsToMake,
    const float mutationRate,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<InputBuilding> &inputBuildings,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const eastl::vector<corex::core::NPolygon> &floodProneAreas,
    const eastl::vector<corex::core::NPolygon> &landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight,
    const bool &keepInfeasibleSolutions)
  {
    IPROF_FUNC;
    std::uniform_real_distribution<float> mutationChanceDistribution{
      0.f, 1.f
    };
    auto children = this->crossoverSolutions(parentA,
                                             parentB,
                                             crossoverType,
                                             boundingArea,
                                             inputBuildings,
                                             keepInfeasibleSolutions);
    children[0].setFitness(computeSolutionFitness(children[0],
                                                  inputBuildings,
                                                  boundingArea,
                                                  flowRates,
                                                  buildingDistanceWeight));
    children[1].setFitness(computeSolutionFitness(children[1],
                                                  inputBuildings,
                                                  boundingArea,
                                                  flowRates,
                                                  buildingDistanceWeight));

    offsprings[numOffsprings] = children[0];

    // Mutation
    float mutationProbability = corex::core::generateRandomReal(
      mutationChanceDistribution);
    if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
      this->mutateSolution(offsprings[numOffsprings],
                           boundingArea,
                           inputBuildings,
                           keepInfeasibleSolutions);
      offsprings[numOffsprings].setFitness(
        computeSolutionFitness(offsprings[numOffsprings],
                               inputBuildings,
                               boundingArea,
                               flowRates,
                               buildingDistanceWeight));
    }

    numOffsprings++;

    // In cases where the population size is not an even number, a child
    // will have to be dropped. As such, we'll only add the second
    // generated child if it has a fitness better than the worst solution
    // in the new generation.
    if (numOffsprings == numOffspringsToMake) {
      auto weakestSolutionIter = std::max_element(
        offsprings.begin(),
        offsprings.end(),
        [](Solution solutionA, Solution solutionB) -> bool {
          return corex::core::floatLessThan(solutionA.getFitness(),
                                            solutionB.getFitness());
        }
      );

      if (corex::core::floatLessThan(children[1].getFitness(),
                                     weakestSolutionIter->getFitness())) {
        int32_t weakestSolutionIndex = std::distance(offsprings.begin(),
                                                     weakestSolutionIter);
        offsprings[weakestSolutionIndex] = children[1];
        offsprings[weakestSolutionIndex].setFitness(
          computeSolutionFitness(offsprings[weakestSolutionIndex],
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));

        float mutationProbability = corex::core::generateRandomReal(
          mutationChanceDistribution);
        if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
          this->mutateSolution(offsprings[weakestSolutionIndex],
                               boundingArea,
                               inputBuildings,
                               keepInfeasibleSolutions);
          offsprings[weakestSolutionIndex].setFitness(
            computeSolutionFitness(offsprings[weakestSolutionIndex],
                                   inputBuildings,
                                   boundingArea,
                                   flowRates,
                                   buildingDistanceWeight));
        }
      }
    } else {
      offsprings[numOffsprings] = children[1];
      offsprings[numOffsprings].setFitness(
        computeSolutionFitness(offsprings[numOffsprings],
                               inputBuildings,
                               boundingArea,
                               flowRates,
                               buildingDistanceWeight));

      float mutationProbability = corex::core::generateRandomReal(
        mutationChanceDistribution);
      if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
        this->mutateSolution(offsprings[numOffsprings],
                             boundingArea,
                             inputBuildings,
                             keepInfeasibleSolutions);
        offsprings[numOffsprings].setFitness(
          computeSolutionFitness(offsprings[numOffsprings],
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
      }

      numOffsprings++;
    }
  }

  eastl::vector<Solution> GA::crossoverSolutions(
    const Solution &solutionA,
    const Solution &solutionB,
    const CrossoverType& type,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<InputBuilding> &inputBuildings,
    const bool &keepInfeasibleSolutions)
  {
    IPROF_FUNC;

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
      case CrossoverType::ARITHMETIC:
        return performArithmeticCrossover(solutionA,
                                          solutionB,
                                          boundingArea,
                                          inputBuildings,
                                          keepInfeasibleSolutions);
      default:
        // TODO: Raise an error since we did not choose a crossover operator.
        break;
    }

    return {};
  }

  void GA::mutateSolution(Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings,
                          const bool& keepInfeasibleSolutions)
  {
    IPROF_FUNC;
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
        std::cout << "Run Buddy-Buddy Mutation.\n";
        applyBuddyBuddyOperator(solution, boundingArea,
                                inputBuildings, -1, -1,
                                keepInfeasibleSolutions);
      },
      [](Solution& solution,
         const corex::core::NPolygon& boundingArea,
         const eastl::vector<InputBuilding>& inputBuildings,
         const bool& keepInfeasibleSolutions)
      {
        std::cout << "Run Shaking Mutation.\n";
        applyShakingOperator(solution, boundingArea,
                             inputBuildings, keepInfeasibleSolutions);
      },
      [](Solution& solution,
         const corex::core::NPolygon& boundingArea,
         const eastl::vector<InputBuilding>& inputBuildings,
         const bool& keepInfeasibleSolutions)
      {
        std::cout << "Run Jiggle Mutation.\n";
        applyJiggleOperator(solution, boundingArea,
                            inputBuildings, keepInfeasibleSolutions);
      }
    };

    Solution tempSolution;
    do {
      tempSolution = solution;
//      const int32_t mutationFuncIndex = cx::getRandomIntUniformly(
//        0, static_cast<int32_t>(mutationFunctions.size() - 1));
      const int32_t mutationFuncIndex = 0;
      mutationFunctions[mutationFuncIndex](tempSolution,
                                           boundingArea,
                                           inputBuildings,
                                           keepInfeasibleSolutions);
    } while (!keepInfeasibleSolutions
             && !isSolutionFeasible(tempSolution, boundingArea,
                                    inputBuildings));
    solution = tempSolution;
  }

  void GA::applyLocalSearch1(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    constexpr int32_t numMovements = 8;
    constexpr float maxShiftAmount = 10.f;
    std::uniform_real_distribution<float> shiftDistrib{ 1.f, maxShiftAmount };
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t)>,
      numMovements> searchFunctions = {
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, -shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the top.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, -shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the bottom.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right then upward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the right and then downward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left and then upward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution, int32_t buildingIndex) -> Solution
      {
        // Move the building to the left and then downward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(buildingIndex, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(buildingIndex, shiftAmount1);

        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      for (int32_t movementID = 0; movementID < numMovements; movementID++) {
        auto altSolution0 = searchFunctions[movementID](solution, i);
        altSolution0.setFitness(
          computeSolutionFitness(altSolution0,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution0);

        auto altSolution1 = altSolution0;
        if (cx::floatEquals(altSolution1.getBuildingAngle(i), 0.f)) {
          altSolution1.setBuildingAngle(i, 90.f);
        } else {
          altSolution1.setBuildingAngle(i, 0.f);
        }
        altSolution1.setFitness(
          computeSolutionFitness(altSolution1,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));

        generatedSolutions.push_back(altSolution1);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](const Solution& solutionA, const Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );
  }

  void GA::applyLocalSearch2(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    constexpr int32_t numMovements = 20;
    constexpr float maxShiftAmount = 10.f;
    std::uniform_real_distribution<float> shiftDistrib{ 1.f, maxShiftAmount };
    static const
    eastl::array<eastl::function<Solution(Solution, int32_t, int32_t)>,
      numMovements> searchFunctions = {
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the right side.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount);
        solution.moveBuildingXPos(building1Idx, shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the left side.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount);
        solution.moveBuildingXPos(building1Idx, -shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings upwards.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount);
        solution.moveBuildingYPos(building1Idx, -shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings downwards.
        float shiftAmount = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount);
        solution.moveBuildingYPos(building1Idx, shiftAmount);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the top-right.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount0);
        solution.moveBuildingXPos(building1Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount1);
        solution.moveBuildingYPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the bottom-right.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount0);
        solution.moveBuildingXPos(building1Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount1);
        solution.moveBuildingYPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the top-left.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount0);
        solution.moveBuildingXPos(building1Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount1);
        solution.moveBuildingYPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift both buildings to the bottom-left.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount0);
        solution.moveBuildingXPos(building1Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount1);
        solution.moveBuildingYPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 upward and building 1 to the right.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 upward and building 1 downward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 upward and building 1 to the left.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the right and building 1 downward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the right and building 1 upward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the right and building 1 to the left.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the left and building 1 downward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the left and building 1 to the right.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 to the left and building 1 upward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building0Idx, -shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 downward and building 1 to the right.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 downward and building 1 to the left.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingXPos(building1Idx, -shiftAmount1);

        return solution;
      },
      [&shiftDistrib](Solution solution,
                      int32_t building0Idx,
                      int32_t building1Idx) -> Solution
      {
        // Shift building 0 downward and building 1 upward.
        float shiftAmount0 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building0Idx, shiftAmount0);

        float shiftAmount1 = corex::core::generateRandomReal(shiftDistrib);
        solution.moveBuildingYPos(building1Idx, -shiftAmount1);

        return solution;
      }
    };

    eastl::vector<Solution> generatedSolutions;
    generatedSolutions.push_back(solution);

    for (int32_t i = 0; i < solution.getNumBuildings() - 1; i++) {
      for (int32_t movementID = 0; movementID < numMovements; movementID++) {
        auto altSolution0 = searchFunctions[movementID](solution, i, i + 1);
        altSolution0.setFitness(
          computeSolutionFitness(altSolution0,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution0);

        auto altSolution1 = altSolution0;
        auto altSolution2 = altSolution0;
        auto altSolution3 = altSolution0;

        if (cx::floatEquals(altSolution1.getBuildingAngle(i), 0.f)) {
          altSolution1.setBuildingAngle(i, 90.f);
        } else {
          altSolution1.setBuildingAngle(i, 0.f);
        }

        if (cx::floatEquals(altSolution2.getBuildingAngle(i + 1), 0.f)) {
          altSolution2.setBuildingAngle(i + 1, 90.f);
        } else {
          altSolution2.setBuildingAngle(i + 1, 0.f);
        }

        if (cx::floatEquals(altSolution3.getBuildingAngle(i), 0.f)) {
          altSolution3.setBuildingAngle(i, 90.f);
        } else {
          altSolution3.setBuildingAngle(i, 0.f);
        }

        if (cx::floatEquals(altSolution3.getBuildingAngle(i + 1), 0.f)) {
          altSolution3.setBuildingAngle(i + 1, 90.f);
        } else {
          altSolution3.setBuildingAngle(i + 1, 0.f);
        }

        altSolution1.setFitness(
          computeSolutionFitness(altSolution1,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution1);

        altSolution2.setFitness(
          computeSolutionFitness(altSolution2,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution2);

        altSolution3.setFitness(
          computeSolutionFitness(altSolution3,
                                 inputBuildings,
                                 boundingArea,
                                 flowRates,
                                 buildingDistanceWeight));
        generatedSolutions.push_back(altSolution3);
      }
    }

    solution = *std::min_element(
      generatedSolutions.begin(),
      generatedSolutions.end(),
      [](const Solution& solutionA, const Solution& solutionB) {
        return solutionA.getFitness() < solutionB.getFitness();
      }
    );
  }
}
