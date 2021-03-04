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
#include <bpt/ds/Solution.hpp>

#include <bpt/HC.hpp>

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
      const CrossoverType crossoverType,
      const SelectionType selectionType,
      const double timeLimit,
      const bool& keepInfeasibleSolutions);
    int32_t getCurrentRunGenerationNumber();
    eastl::vector<float> getRecentRunAverageFitnesses();
    eastl::vector<float> getRecentRunBestFitnesses();
    eastl::vector<float> getRecentRunWorstFitnesses();
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
      const CrossoverType& crossoverType,
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
      const float buildingDistanceWeight,
      const bool& keepInfeasibleSolutions);
    eastl::vector<Solution> crossoverSolutions(
      const CrossoverType& type,
      const Solution& solutionA,
      const Solution& solutionB,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    void mutateSolution(Solution& solution,
                        const corex::core::NPolygon& boundingArea,
                        const eastl::vector<InputBuilding>& inputBuildings,
                        const bool& keepInfeasibleSolutions);
    eastl::unordered_map<int32_t, eastl::vector<int32_t>>
    findFaultyGenes(Solution& solution,
                    const corex::core::NPolygon& boundingArea,
                    const eastl::vector<InputBuilding>& inputBuildings);
    eastl::vector<Solution> performUniformCrossover(
      const Solution& solutionA,
      const Solution& solutionB,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);
    eastl::vector<Solution> performBoxCrossover(
      const Solution& solutionA,
      const Solution& solutionB,
      const corex::core::NPolygon& boundingArea,
      const eastl::vector<InputBuilding>& inputBuildings,
      const bool& keepInfeasibleSolutions);

    template <
      typename RealType,
      typename
      std::enable_if<std::is_floating_point<RealType>::value, bool>::type = true
    > RealType blendTwoValues(RealType a, RealType b)
    {
      RealType contribAmount = cx::getRandomRealUniformly(
        static_cast<RealType>(0), static_cast<RealType>(1));
      return (contribAmount * a) + ((1 - contribAmount) * b);
    }

    int32_t currRunGenerationNumber;
    eastl::vector<float> recentRunAvgFitnesses;
    eastl::vector<float> recentRunBestFitnesses;
    eastl::vector<float> recentRunWorstFitnesses;
    cx::Timer runTimer;
    double recentRunElapsedTime;
    HC hillClimbing;
  };
}

#endif
