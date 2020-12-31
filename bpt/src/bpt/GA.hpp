#ifndef BPT_GA_HPP
#define BPT_GA_HPP

#include <cstdlib>

#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>

#include <bpt/SelectionType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  class GA
  {
  public:
    GA();
    eastl::vector<eastl::vector<Solution>> generateSolutions(
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
      const SelectionType selectionType);
    double getSolutionFitness(
      const Solution& solution,
      const eastl::vector<InputBuilding>& inputBuildings,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight);
    int32_t getCurrentRunGenerationNumber();
    eastl::vector<float> getRecentRunAverageFitnesses();
    eastl::vector<float> getRecentRunBestFitnesses();
    eastl::vector<float> getRecentRunWorstFitnesses();
  private:
    eastl::vector<Solution> generateInitialPopulation(
      const int32_t populationSize,
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<eastl::vector<float>>& flowRates,
      const eastl::vector<corex::core::NPolygon>& floodProneAreas,
      const eastl::vector<corex::core::NPolygon>& landslideProneAreas,
      const float floodProneAreaPenalty,
      const float landslideProneAreaPenalty,
      const float buildingDistanceWeight);
    eastl::array<Solution, 2> selectParents(
      const eastl::vector<Solution>& population,
      const int32_t& tournamentSize,
      const SelectionType& selectionType);
    eastl::array<Solution, 2> runRouletteWheelSelection(
      const eastl::vector<Solution>& population);
    eastl::array<Solution, 2> runTournamentSelection(
      const eastl::vector<Solution>& population,
      const int32_t& tournamentSize);
    void makeTwoParentsBreed(
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
      const float buildingDistanceWeight);
    Solution
    generateRandomSolution(
      const eastl::vector<InputBuilding>& inputBuildings,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<corex::core::Polygon<3>>& boundingAreaTriangles,
      const eastl::vector<float>& triangleAreas);
    eastl::array<Solution, 2>
    crossoverSolutions(const Solution& solutionA,
                       const Solution& solutionB,
                       const corex::core::NPolygon& boundingArea,
                       const eastl::vector<InputBuilding>& inputBuildings);
    void mutateSolution(Solution& solution,
                        const corex::core::NPolygon& boundingArea,
                        const eastl::vector<InputBuilding>& inputBuildings);
    void applyBuddyBuddyMutation(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings);
    void applyShakingMutation(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings);
    void applyJiggleMutation(
      Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings);
    bool isSolutionFeasible(const Solution& solution,
                            const corex::core::NPolygon& boundingArea,
                            const eastl::vector<InputBuilding>& inputBuildings);
    bool doesSolutionHaveNoBuildingsOverlapping(
      const Solution& solution,
      const eastl::vector<InputBuilding>& inputBuildings);
    bool areSolutionBuildingsWithinBounds(
      const Solution& solution,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings);
    int32_t currRunGenerationNumber;
    eastl::vector<float> recentRunAvgFitnesses;
    eastl::vector<float> recentRunBestFitnesses;
    eastl::vector<float> recentRunWorstFitnesses;
  };
}

#endif
