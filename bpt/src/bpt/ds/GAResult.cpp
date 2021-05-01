#include <fstream>

#include <EASTL/vector.h>

#include <bpt/utils.hpp>
#include <bpt/ds/CrossoverType.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/GAResult.hpp>
#include <bpt/ds/Result.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  GAResult::GAResult(const eastl::vector<eastl::vector<Solution>> solutions,
                     const eastl::vector<InputBuilding> inputBuildings,
                     const eastl::vector<double> bestFitnesses,
                     const eastl::vector<double> averageFitnesses,
                     const eastl::vector<double> worstFitnesses,
                     const double elapsedTime,
                     const float mutationRate,
                     const int32_t populationSize,
                     const int32_t numGenerations,
                     const SelectionType selectionType,
                     const int32_t tournamentSize,
                     const CrossoverType crossoverType)
    : Result(solutions,
             inputBuildings,
             bestFitnesses,
             averageFitnesses,
             worstFitnesses,
             elapsedTime)
    , mutationRate(mutationRate)
    , populationSize(populationSize)
    , numGenerations(numGenerations)
    , selectionType(selectionType)
    , tournamentSize(tournamentSize)
    , crossoverType(crossoverType) {}

  void GAResult::customSaveToCSV(std::ofstream& resultsFile)
  {
    resultsFile << "\n";
    resultsFile << "GA Parameters\n"
                << "Mutation Rate:," << this->mutationRate
                << "\n"
                << "Population Size:," << this->populationSize
                << "\n"
                << "No. of Generations:," << this->numGenerations
                << "\n"
                << "Selection Type:,"
                << castToCString(this->selectionType)
                << "\n";

    if (this->selectionType == SelectionType::TS) {
      resultsFile << "Tournament Size:," << this->tournamentSize << "\n";
    }

    resultsFile << "Crossover Type:,"
                << castToCString(this->crossoverType)
                << "\n";
  }
}
