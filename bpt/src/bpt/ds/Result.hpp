#ifndef BPT_DS_RESULT_HPP
#define BPT_DS_RESULT_HPP

#include <fstream>

#include <EASTL/string.h>
#include <EASTL/vector.h>

#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  struct Result
  {
    Result(const eastl::vector<eastl::vector<Solution>> solutions,
           const eastl::vector<InputBuilding> inputBuildings,
           const eastl::vector<double> bestFitnesses,
           const eastl::vector<double> averageFitnesses,
           const eastl::vector<double> worstFitnesses,
           const double elapsedTime);

    const eastl::vector<eastl::vector<Solution>> solutions;
    const eastl::vector<InputBuilding> inputBuildings;
    const eastl::vector<double> bestFitnesses;
    const eastl::vector<double> averageFitnesses;
    const eastl::vector<double> worstFitnesses;
    const double elapsedTime;

    void saveToCSV(const eastl::string& fileName);
  protected:
    virtual void customSaveToCSV(std::ofstream& resultsFile) = 0;
  };
}

#endif
