#include <cassert>
#include <cstring>
#include <iomanip>
#include <iostream>

#include <corex/core/math_functions.hpp>
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
      case AlgorithmType::PSO:
        return "Particle Swarm Optimization";
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
      case CrossoverType::ARITHMETIC:
        return "Arithmetic";
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

  void printVecN(const cx::VecN& vecN)
  {
    for (auto val : vecN) {
      std::cout << std::setw(10) << val;
    }

    std::cout << "\n";
  }

  Solution translateSolutionOrigin(const Solution& solution,
                                   float deltaX,
                                   float deltaY)
  {
    Solution translatedSolution = solution;
    for (int32_t i = 0; i < solution.getNumBuildings(); i++) {
      float oldX = translatedSolution.getBuildingXPos(i);
      float oldY = translatedSolution.getBuildingYPos(i);

      translatedSolution.setBuildingXPos(i, oldX + deltaX);
      translatedSolution.setBuildingYPos(i, oldY + deltaY);
    }

    return translatedSolution;
  }

  cx::VecN clampSolutionVecN(const cx::VecN& vec,
                             float minX,
                             float minY,
                             float maxX,
                             float maxY)
  {
    cx::VecN newVec = vec;
    const int32_t numBuildings = vec.size() / 3;
    for (int32_t i = 0; i < numBuildings; i++) {
      newVec[i * 3] = cx::clamp(vec[i * 3], minX, maxX);
      newVec[(i * 3) + 1] = cx::clamp(vec[(i * 3) + 1], minY, maxY);
    }

    return newVec;
  }

  cx::VecN wrapAroundSolutionVecN(const cx::VecN& vec,
                                  float minX,
                                  float minY,
                                  float maxX,
                                  float maxY)
  {
    cx::VecN newVec = vec;
    const int32_t numBuildings = vec.size() / 3;
    for (int32_t i = 0; i < numBuildings; i++) {
      // Translate the vector a zero origin first from having an origin at the
      // minimum.
      float x = vec[i * 3] - minX;
      float y = vec[(i * 3) + 1] - minY;
      newVec[i * 3] = cx::mod(x, maxX);
      newVec[(i * 3) + 1] = cx::mod(y, maxY);
    }

    return newVec;
  }

  cx::VecN createRandomVector(const int32_t vectorSize,
                              float min, float max)
  {
    cx::VecN randomVecN{vectorSize};
    for (int32_t i = 0; i < vectorSize; i++) {
      randomVecN[i] = cx::getRandomRealUniformly(min, max);
    }

    return randomVecN;
  }
}
