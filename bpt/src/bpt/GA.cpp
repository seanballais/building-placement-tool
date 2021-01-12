#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <random>

#include <EASTL/array.h>
#include <EASTL/functional.h>
#include <EASTL/set.h>
#include <EASTL/vector.h>

#include <corex/core/math_functions.hpp>
#include <corex/core/ds/Line.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/Rectangle.hpp>
#include <corex/core/utils.hpp>

#include <iprof/iprof.hpp>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

#include <bpt/GA.hpp>

namespace bpt
{
  GA::GA()
    : currRunGenerationNumber(-1)
    , recentRunAvgFitnesses()
    , recentRunBestFitnesses()
    , recentRunWorstFitnesses() {}

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
    const SelectionType selectionType)
  {
    assert(flowRates.size() == inputBuildings.size());

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
    this->recentRunAvgFitnesses.push_back(static_cast<float>(fitnessAverage));

    this->recentRunBestFitnesses.push_back(static_cast<float>(
                                             bestSolution.getFitness()));

    this->recentRunWorstFitnesses.push_back(static_cast<float>(
                                              worstSolution.getFitness()));

    const int32_t numOffspringsToMake = populationSize - numPrevGenOffsprings;
    for (int32_t i = 0; i < numGenerations; i++) {
      this->currRunGenerationNumber++;

      int32_t numOffsprings = 0;
      eastl::vector<Solution> newOffsprings(numOffspringsToMake);
      while (numOffsprings < numOffspringsToMake) {
        std::cout << numOffsprings << "\n";
        // Standard Tournament Selection.
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
        this->makeTwoParentsBreed(parentA,
                                  parentB,
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

      // Keep only a set number of offsprings from the previous generation.
      for (int32_t i = numPrevGenOffsprings; i < population.size(); i++) {
        population[i] = newOffsprings[i - numPrevGenOffsprings];
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

      // Might add the local search feature in the future.

      bestSolution.setFitness(this->getSolutionFitness(
        bestSolution,
        inputBuildings,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
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

    this->currRunGenerationNumber = -1;

    return solutions;
  }

  double GA::getSolutionFitness(
    const Solution& solution,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
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

    fitness *= buildingDistanceWeight;

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

  int32_t GA::getCurrentRunGenerationNumber()
  {
    return this->currRunGenerationNumber;
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
    auto boundingAreaTriangles = cx::earClipTriangulate(boundingArea);
    eastl::vector<float> triangleAreas(boundingAreaTriangles.size());
    std::cout << "Triangle Areas\n";
    for (int32_t i = 0; i < boundingAreaTriangles.size(); i++) {
      triangleAreas[i] = cx::getPolygonArea(boundingAreaTriangles[i]);
      std::cout << triangleAreas[i] << "\n";
    }

    eastl::vector<Solution> population(populationSize);
    std::cout << "|| Generating Initial Population..." << std::endl;
    for (int32_t i = 0; i < populationSize; i++) {
      std::cout << "Generating solution #" << i << "..." << std::endl;
      population[i] = this->generateRandomSolution(inputBuildings,
                                                   boundingArea,
                                                   boundingAreaTriangles,
                                                   triangleAreas);
      population[i].setFitness(
        this->getSolutionFitness(
          population[i],
          inputBuildings,
          flowRates,
          floodProneAreas,
          landslideProneAreas,
          floodProneAreaPenalty,
          landslideProneAreaPenalty,
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
      case SelectionType::RWS:
        parents = this->runRouletteWheelSelection(population);
      case SelectionType::TS:
        parents = this->runTournamentSelection(population, tournamentSize);
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
    const Solution& parentA,
    const Solution& parentB,
    eastl::vector<Solution>& offsprings,
    int32_t& numOffsprings,
    const int32_t numOffspringsToMake,
    const float mutationRate,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const eastl::vector<corex::core::NPolygon>& floodProneAreas,
    const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
    const float floodProneAreaPenalty,
    const float landslideProneAreaPenalty,
    const float buildingDistanceWeight)
  {
    IPROF_FUNC;
    std::uniform_real_distribution<float> mutationChanceDistribution{
      0.f, 1.f
    };
    offsprings[numOffsprings] = this->crossoverSolutions(parentA,
                                                         parentB,
                                                         boundingArea,
                                                         inputBuildings);
    offsprings[numOffsprings].setFitness(this->getSolutionFitness(
      offsprings[numOffsprings],
      inputBuildings,
      flowRates,
      floodProneAreas,
      landslideProneAreas,
      floodProneAreaPenalty,
      landslideProneAreaPenalty,
      buildingDistanceWeight));

    // Mutation
    float mutationProbability = corex::core::generateRandomReal(
      mutationChanceDistribution);
    if (corex::core::floatLessThan(mutationProbability, mutationRate)) {
      this->mutateSolution(offsprings[numOffsprings],
                           boundingArea,
                           inputBuildings);
      offsprings[numOffsprings].setFitness(this->getSolutionFitness(
        offsprings[numOffsprings],
        inputBuildings,
        flowRates,
        floodProneAreas,
        landslideProneAreas,
        floodProneAreaPenalty,
        landslideProneAreaPenalty,
        buildingDistanceWeight));
    }

    numOffsprings++;
  }

  Solution
  GA::generateRandomSolution(
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<cx::Polygon<3>>& boundingAreaTriangles,
    const eastl::vector<float>& triangleAreas)
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
          const cx::Polygon<3>& triangle = cx::selectRandomItemWithWeights(
            boundingAreaTriangles,
            triangleAreas);
          cx::Point newBuildingPos = cx::getRandomPointInTriangle(triangle);

          buildingPos.x = newBuildingPos.x;
          buildingPos.y = newBuildingPos.y;
          buildingRotation = corex::core::generateRandomReal(
            rotationDistribution);
          buildingRect.x = buildingPos.x;
          buildingRect.y = buildingPos.y;
          buildingRect.angle = buildingRotation;

          solution.setBuildingXPos(i, buildingPos.x);
          solution.setBuildingYPos(i, buildingPos.y);
          solution.setBuildingRotation(i, buildingRotation);
        } while (!isRectWithinNPolygon(buildingRect, boundingArea)
                 || !this->doesSolutionHaveNoBuildingsOverlapping(
                      solution,
                      inputBuildings));
      }
    } while (!this->isSolutionFeasible(solution, boundingArea, inputBuildings));

    return solution;
  }

  Solution GA::crossoverSolutions(
    const Solution& solutionA,
    const Solution& solutionB,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    IPROF_FUNC;
    std::uniform_int_distribution<int32_t> parentDistrib{ 0, 1 };
    std::uniform_real_distribution<float> normalScaleDistrib{ 0.f, 1.f };
    std::uniform_real_distribution<float> angleDistrib{ 0.f, 360.f };
    int32_t numBuildings = solutionA.getNumBuildings();

    // Prevent unnecessary copying of the parents.
    eastl::array<const Solution* const, 2> parents{ &solutionA, &solutionB };
    Solution child{numBuildings};
    do {
      IPROF("Crossover Main");
      // Perform Box Crossover.
      for (int32_t i = 0; i < numBuildings; i++) {
        // Compute child's new x position.
        float lowerXBound = std::min(parents[0]->getBuildingXPos(i),
                                     parents[1]->getBuildingXPos(i));
        float upperXBound = std::max(parents[0]->getBuildingXPos(i),
                                     parents[1]->getBuildingXPos(i));
        child.setBuildingXPos(
          i,
          cx::getRandomRealUniformly(lowerXBound, upperXBound));

        // Compute child's new y position.
        float lowerYBound = std::min(parents[0]->getBuildingYPos(i),
                                     parents[1]->getBuildingYPos(i));
        float upperYBound = std::max(parents[0]->getBuildingYPos(i),
                                     parents[1]->getBuildingYPos(i));
        child.setBuildingYPos(
          i,
          cx::getRandomRealUniformly(lowerYBound, upperYBound));

        // Compute child's new angle.
        float lowerAngleBound = std::min(parents[0]->getBuildingRotation(i),
                                         parents[1]->getBuildingRotation(i));
        float upperAngleBound = std::max(parents[0]->getBuildingRotation(i),
                                         parents[1]->getBuildingRotation(i));
        child.setBuildingRotation(
          i,
          cx::getRandomRealUniformly(lowerAngleBound, upperAngleBound));
      }

      std::cout << "-------------\n";
      for (const auto& [ k, _ ] : this->findFaultyGenes(child, boundingArea,
                                                        inputBuildings)) {
        std::cout << k << ", ";
      }
      std::cout << "\n";

      if (!this->isSolutionFeasible(child, boundingArea, inputBuildings)) {
        std::cout << "######################\n";
        for (int32_t i = 0; i < child.getNumBuildings(); i++) {
          std::cout << "x: " << child.getBuildingXPos(i) << "\n"
                    << "y: " << child.getBuildingYPos(i) << "\n"
                    << "angle: " << child.getBuildingRotation(i) << "\n";
          std::cout << "*****************************\n";
        }
        std::cout << "######################\n";
      }
    } while (!this->isSolutionFeasible(child, boundingArea, inputBuildings));

    return child;
  }

  void GA::mutateSolution(Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings)
  {
    IPROF_FUNC;
    eastl::array<eastl::function<void(Solution&,
                                 const corex::core::NPolygon&,
                                 const eastl::vector<InputBuilding>&)>,
                 3> mutationFunctions = {
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings)
      {
        this->applyBuddyBuddyMutation(solution, boundingArea, inputBuildings);
      },
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings)
      {
        this->applyShakingMutation(solution, boundingArea, inputBuildings);
      },
      [this](Solution& solution,
             const corex::core::NPolygon& boundingArea,
             const eastl::vector<InputBuilding>& inputBuildings)
      {
        this->applyJiggleMutation(solution, boundingArea, inputBuildings);
      }
    };

    std::uniform_int_distribution<const int32_t> numMutationsDistrib{
      0, static_cast<int32_t>(mutationFunctions.size() - 1)
    };
    const int32_t mutationFuncIndex = corex::core::generateRandomInt(
      numMutationsDistrib);
    mutationFunctions[mutationFuncIndex](solution,
                                         boundingArea,
                                         inputBuildings);
  }

  void GA::repairSolution(Solution& solution,
                          const corex::core::NPolygon& boundingArea,
                          const eastl::vector<InputBuilding>& inputBuildings)
  {
    IPROF_FUNC;
    auto faultyGenes = this->findFaultyGenes(solution,
                                             boundingArea,
                                             inputBuildings);
    // Invalidate faulty genes first so that they won't have an effect on the
    // solution's fitness during repair.
    for (const auto& [ key, value ] : faultyGenes) {
      solution.invalidateBuildingData(key);
    }

    for (const auto& [ key, value ] : faultyGenes) {
      solution.validateBuildingData(key);
      this->applyBuddyBuddyMutation(solution,
                                    boundingArea,
                                    inputBuildings,
                                    -1,
                                    key);
    }
  }

  eastl::unordered_map<int32_t, eastl::vector<int32_t>>
  GA::findFaultyGenes(Solution& solution,
                      const corex::core::NPolygon& boundingArea,
                      const eastl::vector<InputBuilding>& inputBuildings)
  {
    eastl::unordered_map<int32_t, eastl::vector<int32_t>> faultyGenes;

    // Find overlapping buildings.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      auto building0 = corex::core::Rectangle{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };

      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        auto building1 = corex::core::Rectangle{
          solution.getBuildingXPos(j),
          solution.getBuildingYPos(j),
          inputBuildings[j].width,
          inputBuildings[j].length,
          solution.getBuildingRotation(j)
        };
        if (corex::core::areTwoRectsIntersecting(building0, building1)) {
          auto iter = faultyGenes.insert(i);
          iter.first->second.push_back(j);
        }
      }
    }

    // Find buildings overlapping the bounding area.
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      auto buildingRect = corex::core::Rectangle{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };

      if (!corex::core::isRectWithinNPolygon(buildingRect, boundingArea)) {
        faultyGenes.insert(i);
      }
    }

    return faultyGenes;
  }

  void GA::applyBuddyBuddyMutation(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings,
    const int32_t staticBuildingIndex,
    const int32_t dynamicBuildingIndex)
  {
    IPROF_FUNC;
    std::uniform_int_distribution<int32_t> buildingDistrib{
      0, static_cast<int32_t>(inputBuildings.size() - 1)
    };
    std::uniform_int_distribution<int32_t> buddySideDistrib{ 0, 3 };
    std::uniform_int_distribution<int32_t> relOrientationDistrib{ 0, 1 };
    std::uniform_real_distribution<float> normalizedDistrib{ 0, 1 };

    Solution tempSolution;
    do {
      IPROF("Buddy-Buddy Mutation Main");
      tempSolution = solution;

      // Let's just do the Buddy-Buddy Mutation for now.
      int32_t staticBuddy = 0;
      int32_t dynamicBuddy = 0; // The buddy to be moved.
      do {
        if (staticBuildingIndex == -1) {
          staticBuddy = corex::core::generateRandomInt(buildingDistrib);
        } else {
          staticBuddy = staticBuildingIndex;
        }

        if (dynamicBuildingIndex == -1) {
          dynamicBuddy = corex::core::generateRandomInt(buildingDistrib);
        } else {
          dynamicBuddy = dynamicBuildingIndex;
        }
      } while (staticBuddy == dynamicBuddy
               || !solution.isBuildingDataUsable(staticBuddy)
               || !solution.isBuildingDataUsable(dynamicBuddy));

      auto staticBuddyRect = corex::core::Rectangle{
        solution.getBuildingXPos(staticBuddy),
        solution.getBuildingYPos(staticBuddy),
        inputBuildings[staticBuddy].width,
        inputBuildings[staticBuddy].length,
        solution.getBuildingRotation(staticBuddy)
      };
      auto buddyPoly = corex::core::convertRectangleToPolygon(staticBuddyRect);

      const int32_t buddySide = corex::core::generateRandomInt(
        buddySideDistrib);

      corex::core::Line contactLine;
      switch (buddySide) {
        case 0:
          contactLine = corex::core::Line{
            { buddyPoly.vertices[0].x, buddyPoly.vertices[0].y },
            { buddyPoly.vertices[1].x, buddyPoly.vertices[1].y }
          };
          break;
        case 1:
          contactLine = corex::core::Line{
            { buddyPoly.vertices[1].x, buddyPoly.vertices[1].y },
            { buddyPoly.vertices[2].x, buddyPoly.vertices[2].y }
          };
          break;
        case 2:
          contactLine = corex::core::Line{
            { buddyPoly.vertices[2].x, buddyPoly.vertices[2].y },
            { buddyPoly.vertices[3].x, buddyPoly.vertices[3].y }
          };
          break;
        case 3:
          contactLine = corex::core::Line{
            { buddyPoly.vertices[3].x, buddyPoly.vertices[3].y },
            { buddyPoly.vertices[0].x, buddyPoly.vertices[0].y }
          };
          break;
        default:
          break;
      }

      auto contactLineVec = corex::core::lineToVec(contactLine);
      const int32_t orientation = corex::core::generateRandomInt(
        relOrientationDistrib);
      float distContactToBuddyCenter = 0.f;

      // Length to add to both ends of the contact line vector to allow the
      // edges in the dynamic buddy perpendicular to contact line to be in line
      // with those edges parallel to it in the static buddy.
      float extLength = 0.f;

      float contactLineAngle = corex::core::vec2Angle(contactLineVec);
      float dynamicBuddyAngle;
      if (orientation == 0) {
        // The dynamic buddy will be oriented parallel to the contact line, if
        // width > length. Perpendicular, otherwise.
        distContactToBuddyCenter = inputBuildings[dynamicBuddy].width / 2.f;
        extLength = inputBuildings[dynamicBuddy].length / 2.f;
        dynamicBuddyAngle = contactLineAngle;
      } else if (orientation == 1) {
        // The dynamic buddy will be oriented perpendicular to the contact line,
        // if length > width. Parallel, otherwise.
        distContactToBuddyCenter = inputBuildings[dynamicBuddy].length / 2.f;
        extLength = inputBuildings[dynamicBuddy].width / 2.f;
        dynamicBuddyAngle = contactLineAngle + 45.f;
      }

      // Adjust the distance of the dynamic buddy centroid to the contact line
      // by a small amount to prevent intersection of buildings.
      distContactToBuddyCenter += 0.0001f;

      auto buddyMidptRelContactLine = corex::core::rotateVec2(
        corex::core::Vec2{0.f, extLength * 2 }, contactLineAngle)
        + contactLineVec;
      auto buddyMidptRelContactLineStart = corex::core::rotateVec2(
        corex::core::Vec2{ 0.f, -extLength }, contactLineAngle)
        + contactLine.start;

      const float lineWidthModifier = corex::core::generateRandomReal(
        normalizedDistrib);

      corex::core::Point dynamicBuddyPos{
        ((buddyMidptRelContactLine * lineWidthModifier)
         + corex::core::vec2Perp(
          corex::core::rotateVec2(
            corex::core::Vec2{ 0.f, distContactToBuddyCenter },
            contactLineAngle)))
        + buddyMidptRelContactLineStart
      };

      tempSolution.setBuildingXPos(dynamicBuddy, dynamicBuddyPos.x);
      tempSolution.setBuildingYPos(dynamicBuddy, dynamicBuddyPos.y);
      tempSolution.setBuildingRotation(dynamicBuddy, dynamicBuddyAngle);
    } while (!this->isSolutionFeasible(tempSolution,
                                       boundingArea,
                                       inputBuildings));
    solution = tempSolution;
  }

  void GA::applyShakingMutation(
    Solution& solution,
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

  void GA::applyJiggleMutation(
    Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    Solution tempSolution;
    do {
      tempSolution = solution;

      constexpr int32_t numMovements = 8;
      constexpr float maxShiftAmount = 1.f;
      constexpr float maxRotShiftAmount = 5.f;
      std::uniform_real_distribution<float> shiftDistrib{ 0, maxShiftAmount };
      std::uniform_int_distribution<int32_t> buildingIndexDistrib{
        0, static_cast<int32_t>(inputBuildings.size() - 1)
      };
      std::uniform_real_distribution<float> rotShiftDistrib{
        -maxRotShiftAmount,
        maxRotShiftAmount
      };
      static const
      eastl::array<eastl::function<Solution(Solution, int32_t)>,
        numMovements> jiggleFunctions = {
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
      std::uniform_int_distribution<int32_t> jiggleFuncDistrib{
        0, static_cast<int32_t>(jiggleFunctions.size() - 1)
      };

      const int32_t targetBuildingIndex = corex::core::generateRandomInt(
        buildingIndexDistrib);
      const int32_t jiggleFuncIndex = corex::core::generateRandomInt(
        jiggleFuncDistrib);

      tempSolution = jiggleFunctions[jiggleFuncIndex](tempSolution,
                                                      targetBuildingIndex);

      const float rotDelta = corex::core::generateRandomReal(rotShiftDistrib);
      const float newRot = tempSolution.getBuildingRotation(targetBuildingIndex)
                           + rotDelta;
      tempSolution.setBuildingRotation(targetBuildingIndex, newRot);
    } while (!this->isSolutionFeasible(tempSolution,
                                       boundingArea,
                                       inputBuildings));

    solution = tempSolution;
  }

  bool GA::isSolutionFeasible(
    const Solution& solution,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<InputBuilding>& inputBuildings)
  {
    IPROF_FUNC;
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
      if (!solution.isBuildingDataUsable(i)) {
        continue;
      }

      corex::core::Rectangle building0 = corex::core::Rectangle{
        solution.getBuildingXPos(i),
        solution.getBuildingYPos(i),
        inputBuildings[i].width,
        inputBuildings[i].length,
        solution.getBuildingRotation(i)
      };

      for (int32_t j = i + 1; j < solution.getNumBuildings(); j++) {
        if (!solution.isBuildingDataUsable(j)) {
          continue;
        }

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
      if (!solution.isBuildingDataUsable(i)) {
        continue;
      }

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
