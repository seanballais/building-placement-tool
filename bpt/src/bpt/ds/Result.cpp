#include <fstream>
#include <sstream>

#include <EASTL/string.h>

#include <corex/core/utils.hpp>

#include <bpt/ds/Result.hpp>
#include <bpt/ds/Time.hpp>

namespace bpt
{
  Result::Result(const eastl::vector<eastl::vector<Solution>> solutions,
         const eastl::vector<InputBuilding> inputBuildings,
         const eastl::vector<double> bestFitnesses,
         const eastl::vector<double> averageFitnesses,
         const eastl::vector<double> worstFitnesses,
         const double elapsedTime)
    : solutions(solutions)
    , inputBuildings(inputBuildings)
    , bestFitnesses(bestFitnesses)
    , averageFitnesses(averageFitnesses)
    , worstFitnesses(worstFitnesses)
    , elapsedTime(elapsedTime) {}

  void Result::saveToCSV(const eastl::string& fileName)
  {
    std::stringstream resultsFileRelPath;
    resultsFileRelPath << "data/" << cx::eaStrToStdStr(fileName) << ".csv";

    std::filesystem::path resultsFilePath = corex::core::getBinFolder()
                                            / resultsFileRelPath.str();
    std::cout << "Filepath: " << resultsFilePath.string() << "\n";
    std::ofstream resultsFile;
    resultsFile.open(resultsFilePath.string());

    // Put the header of the CSV
    for (int32_t i = 0; i < this->solutions.size(); i++) {
      resultsFile << ", Iteration #" << i;
    }
    resultsFile << "\n";

    resultsFile << "Average Fitness";
    for (const double& fitness : this->averageFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "Best Fitness";
    for (const double& fitness : this->bestFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "Worst Fitness";
    for (const double& fitness : this->worstFitnesses) {
      resultsFile << "," << fitness;
    }
    resultsFile << "\n";

    resultsFile << "\n";
    resultsFile << "Input Buildings" << "\n"
                << "Width,Length" << "\n";
    for (const InputBuilding& inputBuilding : this->inputBuildings) {
      resultsFile << inputBuilding.width << ","
                  << inputBuilding.length << "\n";
    }

    resultsFile << "\n";
    resultsFile << "Best Solution:" << "\n"
                << "x,y,Rotation" << "\n";
    for (int32_t i = 0;
         i < this->solutions.back()[0].getNumBuildings();
         i++) {
      resultsFile << this->solutions.back()[0].getBuildingXPos(i) << ","
                  << this->solutions.back()[0].getBuildingYPos(i) << ","
                  << this->solutions.back()[0].getBuildingAngle(i) << "\n";
    }

    Time elapsedTimeObj{ this->elapsedTime };

    resultsFile << "\n";
    resultsFile << "Run Elapsed Time:,"
                << std::setfill('0') << std::setw(2) << elapsedTimeObj.hours
                << ":"
                << std::setfill('0') << std::setw(2) << elapsedTimeObj.minutes
                << ":"
                << std::setfill('0') << std::setw(2) << elapsedTimeObj.seconds
                << "\n"
                << "\n";

    this->customSaveToCSV(resultsFile);

    if (resultsFile.is_open()) {
      resultsFile.close();
    }
  }
}
