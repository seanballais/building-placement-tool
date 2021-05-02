#ifndef BPT_GA_HPP
#define BPT_GA_HPP

#include <cstdlib>

#include <EASTL/unordered_map.h>
#include <EASTL/vector.h>

#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/Timer.hpp>
#include <corex/core/utils.hpp>

#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/GAResult.hpp>
#include <bpt/ds/Solution.hpp>

#include <bpt/HC.hpp>

namespace bpt
{
  class GA
  {
  public:
    GA();
    GAResult generateSolutions(
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
      const bool& keepInfeasibleSolutions);
    int32_t getCurrentRunIterationNumber();
    eastl::vector<double> getRecentRunAverageFitnesses();
    eastl::vector<double> getRecentRunBestFitnesses();
    eastl::vector<double> getRecentRunWorstFitnesses();
    double getRecentRunElapsedTime();
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
    eastl::array<Solution, 2> runRankedSelection(
      eastl::vector<Solution> population);
    eastl::array<Solution, 2> runTournamentSelection(
      const eastl::vector<Solution>& population,
      const int32_t& tournamentSize);
    void makeTwoParentsBreed(
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
      const bool &keepInfeasibleSolutions);
    eastl::vector<Solution> crossoverSolutions(
      const Solution &solutionA,
      const Solution &solutionB,
      const CrossoverType& type,
      const corex::core::NPolygon &boundingArea,
      const eastl::vector<InputBuilding> &inputBuildings,
      const bool &keepInfeasibleSolutions);
    void mutateSolution(Solution& solution,
                        const corex::core::NPolygon& boundingArea,
                        const eastl::vector<InputBuilding>& inputBuildings,
                        const bool& keepInfeasibleSolutions);

    int32_t currRunGenerationNumber;
    eastl::vector<double> recentRunAvgFitnesses;
    eastl::vector<double> recentRunBestFitnesses;
    eastl::vector<double> recentRunWorstFitnesses;
    cx::Timer runTimer;
    double recentRunElapsedTime;
    HC hillClimbing;
  };
}

#endif
