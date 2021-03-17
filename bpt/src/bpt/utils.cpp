#include <cassert>
#include <cstring>

#include <corex/core/ds/VecN.hpp>

#include <bpt/utils.hpp>
#include <bpt/ds/AlgorithmType.hpp>
#include <bpt/ds/SelectionType.hpp>
#include <bpt/ds/Solution.hpp>

namespace bpt
{
  const char* castToCString(AlgorithmType type)
  {
    switch (type) {
      case AlgorithmType::GA:
        return "Genetic Algorithm";
      case AlgorithmType::GWO:
        return "Grey Wolf Optimization";
      case AlgorithmType::HC:
        return "Naive Hill Climbing";
    }
  }

  const char* castToCString(CrossoverType type)
  {
    switch (type) {
      case CrossoverType::NONE:
        return "None";
      case CrossoverType::UNIFORM:
        return "Uniform";
      case CrossoverType::BOX:
        return "Box";
    }
  }

  const char* castToCString(SelectionType type)
  {
    switch (type) {
      case SelectionType::NONE:
        return "None";
      case SelectionType::RS:
        return "Ranked Selection";
      case SelectionType::RWS:
        return "Roulette Wheel";
      case SelectionType::TS:
        return "Tournament";
    }
  }

  CrossoverType cStringToCrossoverType(const char* str)
  {
    if (strcmp(str, "None") == 0) {
      return CrossoverType::NONE;
    } else if (strcmp(str, "Uniform") == 0) {
      return CrossoverType::UNIFORM;
    } else if (strcmp(str, "Box") == 0) {
      return CrossoverType::BOX;
    } else {
      assert(false);
    }
  }

  cx::VecN convertSolutionToVecN(const Solution& solution)
  {
    const int32_t vecNSize = solution.getNumBuildings() * 3;
    cx::VecN vecN{vecNSize};
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      vecN[i * 3] = solution.getBuildingXPos(i);
      vecN[(i * 3) + 1] = solution.getBuildingYPos(i);
      vecN[(i * 3) + 2] = solution.getBuildingAngle(i);
    }

    return vecN;
  }

  Solution convertVecNToSolution(const cx::VecN& vecN)
  {
    assert((vecN.size() % 3) == 0);

    int32_t numBuildings = static_cast<int32_t>(vecN.size() / 3);
    Solution solution{numBuildings};
    for (int32_t i = 0; i < numBuildings; i++) {
      solution.setBuildingXPos(i, vecN[i * 3]);
      solution.setBuildingYPos(i, vecN[(i * 3) + 1]);
      solution.setBuildingAngle(i, vecN[(i * 3) + 2]);
    }

    return solution;
  }
}
